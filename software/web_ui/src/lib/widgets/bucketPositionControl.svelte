<script lang="ts" module>
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Bucket Position Control';
	export const description = 'View current bucket position and call preset positions';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = false;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import type { CanvasApi } from './shared/canvasWrapper.svelte';
	import CanvasWrapper from './shared/canvasWrapper.svelte';
	import type {
		BoolType,
		JointStateType,
		StampedHeaderType,
		TriggerType
	} from '$lib/scripts/rosTypes';
	import Button from '$lib/components/ui/button/button.svelte';
	import * as ButtonGroup from '$lib/components/ui/button-group/button-group.svelte';
	import Input from '$lib/components/ui/input/input.svelte';
	import Label from '$lib/components/ui/label/label.svelte';

	interface ValuesType {
		positions: Array<number>;
		velocities: Array<number>;
		accelerations: Array<number>;
		effort: Array<number>;
		time_from_start: { sec: number; nanosec: number };
	}

	interface ActionFeedbackType {
		header: StampedHeaderType;
		joint_names: Array<string>;
		actual: ValuesType;
		desired: ValuesType;
		error: ValuesType;
	}

	interface ActionResultType {
		error_code: number;
		error_string: string;
	}

	interface ActionGoalType {
		trajectory: {
			joint_names: Array<string>;
			points: Array<Partial<ValuesType>>;
		};
	}

	const WIDTH = 900;
	const HEIGHT = 600;

	const SCALE = 0.6;

	const HALF_BASE_WIDTH = 120 * SCALE;
	const ACTUATOR_SLEEVE_LENGTH = 350 * SCALE;
	const ACTUATOR_ARM_LENGTH = 286 * SCALE;
	const CENTRAL_MEMBER_LENGTH = 750 * SCALE;
	const LIFT_ACTUATOR_OFFSET = 117 * SCALE;
	const TILT_ACTUATOR_OFFSET = 250 * SCALE;
	const BUCKET_BASE = 140 * SCALE;
	const BUCKET_DEPTH = 100 * SCALE;
	const X_OFFSET = 400 * SCALE;
	const angleOffset = { jaw: 180, tilt: 255, lift: 197 };

	let presets = $state<
		Array<{
			name: string;
			jaw: number;
			tilt: number;
			lift: number;
		}>
	>([
		{ name: 'Home', jaw: 0, tilt: 0, lift: 0 },
		{ name: 'Preset 1', jaw: 10, tilt: 0, lift: 0 },
		{ name: 'Preset 2', jaw: 60, tilt: 20, lift: 30 },
		{ name: 'Max', jaw: 45, tilt: 90, lift: 0 },
		{ name: 'Min', jaw: 0, tilt: -50, lift: 40 }
	]);
	let currentAngles = $state({ jaw: 0, tilt: 0, lift: 0 });
	let targetAngles = $state({ jaw: 0, tilt: 0, lift: 0 });
	let inputAngles = $state({ jaw: 0, tilt: 0, lift: 0 });
	let isOperatorControl = $state(false);
	let lastGoalId = $state<string | null>(null);
	let errorMessage = $state<string | null>(null);

	// topic which publishes the current joint angles
	let jointState: ROSLIB.Topic<JointStateType> | null = null;
	// Action client for sending a new trajectory
	let action: ROSLIB.Action<ActionGoalType, ActionFeedbackType, ActionResultType> | null = null;
	// service to give control of the bucket back to the mission planner
	let rearm: ROSLIB.Service<TriggerType['request'], TriggerType['response']> | null = null;
	// topic which publishes true when the bucket is under manul control
	let operatorControl: ROSLIB.Topic<BoolType> | null = null;

	const polarToCartesian = (radius: number, degrees: number, xOffset: number, yOffset: number) => {
		const radians = (degrees * Math.PI) / 180;
		return {
			x: radius * Math.cos(radians) + xOffset,
			y: radius * Math.sin(radians) + yOffset
		};
	};

	const partialLine = (x1: number, y1: number, x2: number, y2: number, fraction: number) => {
		return {
			x: x1 + (x2 - x1) * fraction,
			y: y1 + (y2 - y1) * fraction
		};
	};

	const distance = (x1: number, y1: number, x2: number, y2: number) => {
		return Math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);
	};

	// Real joint angles -> drawing angles. Presets need this too.
	type Angles = { jaw: number; tilt: number; lift: number };
	const toVirtual = (a: Angles): Angles => ({
		jaw: -(a.jaw + angleOffset.jaw),
		tilt: a.tilt + angleOffset.tilt,
		lift: a.lift + angleOffset.lift
	});

	const drawBucket = (
		canvas: CanvasApi,
		angles: { jaw: number; tilt: number; lift: number },
		opacity: number
	) => {
		canvas.setOpacity(opacity);
		// base plate center is reference point
		const baseCenter = { x: X_OFFSET, y: 0 };
		const baseTop = { x: X_OFFSET, y: HALF_BASE_WIDTH };
		const baseBottom = { x: X_OFFSET, y: -HALF_BASE_WIDTH };
		canvas.drawLine(baseCenter.x, baseCenter.y, baseTop.x, baseTop.y, 'grey', 4);
		canvas.drawLine(baseCenter.x, baseCenter.y, baseBottom.x, baseBottom.y, 'grey', 4);
		// center member
		const memberEndPoint = polarToCartesian(
			CENTRAL_MEMBER_LENGTH,
			angles.lift,
			baseTop.x,
			baseTop.y
		);
		canvas.drawLine(baseTop.x, baseTop.y, memberEndPoint.x, memberEndPoint.y, 'lightgrey', 4);
		// bucket
		const bucketTop = polarToCartesian(
			BUCKET_BASE,
			angles.tilt + angles.lift,
			memberEndPoint.x,
			memberEndPoint.y
		);
		const jawRightBase = polarToCartesian(
			BUCKET_BASE,
			angles.jaw + angles.tilt + angles.lift,
			bucketTop.x,
			bucketTop.y
		);
		const jawLeftBase = polarToCartesian(
			BUCKET_DEPTH,
			angles.jaw + angles.tilt + angles.lift - 90,
			jawRightBase.x,
			jawRightBase.y
		);
		canvas.drawLine(memberEndPoint.x, memberEndPoint.y, bucketTop.x, bucketTop.y, 'dimgrey', 4);
		canvas.fillTriangle(
			bucketTop.x,
			bucketTop.y,
			jawRightBase.x,
			jawRightBase.y,
			jawLeftBase.x,
			jawLeftBase.y,
			'dimgrey'
		);

		// lift actuator
		const liftEndPoint = polarToCartesian(
			CENTRAL_MEMBER_LENGTH - LIFT_ACTUATOR_OFFSET,
			angles.lift,
			baseTop.x,
			baseTop.y
		);
		const liftActuatorLength = distance(baseBottom.x, baseBottom.y, liftEndPoint.x, liftEndPoint.y);
		const liftSleeveEnd = partialLine(
			baseBottom.x,
			baseBottom.y,
			liftEndPoint.x,
			liftEndPoint.y,
			ACTUATOR_SLEEVE_LENGTH / liftActuatorLength
		);
		const liftArmEnd = partialLine(
			liftEndPoint.x,
			liftEndPoint.y,
			baseBottom.x,
			baseBottom.y,
			ACTUATOR_ARM_LENGTH / liftActuatorLength
		);
		canvas.drawLine(liftArmEnd.x, liftArmEnd.y, liftEndPoint.x, liftEndPoint.y, 'lightgrey', 4);
		canvas.drawLine(baseBottom.x, baseBottom.y, liftSleeveEnd.x, liftSleeveEnd.y, 'dimgrey', 16);

		// tilt actuator
		const tiltEndPoint = polarToCartesian(TILT_ACTUATOR_OFFSET, angles.lift, baseTop.x, baseTop.y);
		const tiltActuatorLength = distance(bucketTop.x, bucketTop.y, tiltEndPoint.x, tiltEndPoint.y);
		const tiltSleeveEnd = partialLine(
			tiltEndPoint.x,
			tiltEndPoint.y,
			bucketTop.x,
			bucketTop.y,
			ACTUATOR_SLEEVE_LENGTH / tiltActuatorLength
		);
		const tiltArmEnd = partialLine(
			bucketTop.x,
			bucketTop.y,
			tiltEndPoint.x,
			tiltEndPoint.y,
			ACTUATOR_ARM_LENGTH / tiltActuatorLength
		);
		canvas.drawLine(tiltArmEnd.x, tiltArmEnd.y, bucketTop.x, bucketTop.y, 'lightgrey', 4);
		canvas.drawLine(
			tiltEndPoint.x,
			tiltEndPoint.y,
			tiltSleeveEnd.x,
			tiltSleeveEnd.y,
			'dimgrey',
			16
		);
	};

	const drawCentered = (c: CanvasApi, angles: Angles, opacity: number) => {
		// Pass 1: record the min and max y of everything drawn
		let min = Infinity;
		let max = -Infinity;
		const track = (...ys: number[]) => {
			min = Math.min(min, ...ys);
			max = Math.max(max, ...ys);
		};
		const probe = {
			setOpacity() {},
			drawLine: (_x1: number, y1: number, _x2: number, y2: number) => track(y1, y2),
			fillTriangle: (_x1: number, y1: number, _x2: number, y2: number, _x3: number, y3: number) =>
				track(y1, y2, y3),
			fillRect: (_x1: number, y1: number, _x2: number, y2: number) => track(y1, y2)
		} as unknown as CanvasApi;
		drawBucket(probe, angles, opacity);

		// Pass 2: draw for real, shifted so the drawing is centred on y = 0
		const dy = -(min + max) / 2;
		drawBucket(
			{
				...c,
				drawLine: (x1, y1, x2, y2, col, w) => c.drawLine(x1, y1 + dy, x2, y2 + dy, col, w),
				fillTriangle: (x1, y1, x2, y2, x3, y3, col) =>
					c.fillTriangle(x1, y1 + dy, x2, y2 + dy, x3, y3 + dy, col),
				fillRect: (x1, y1, x2, y2, col) => c.fillRect(x1, y1 + dy, x2, y2 + dy, col)
			},
			angles,
			opacity
		);
	};

	// ondraw runs on mount, on resize, and whenever the $state it reads changes.
	const drawMain = (c: CanvasApi) => {
		if (!isOperatorControl) {
			drawBucket(c, toVirtual(targetAngles), 0.3);
		}
		drawBucket(c, toVirtual(currentAngles), 1);
	};

	// handle ros2 connection
	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			jointState = new ROSLIB.Topic({
				ros: ros,
				name: '/joint_states',
				messageType: 'sensor_msgs/JointState'
			});
			action = new ROSLIB.Action({
				ros: ros,
				name: '/bucket_trajectory_controller/follow_joint_trajectory',
				actionType: 'control_msgs/action/FollowJointTrajectory'
			});
			rearm = new ROSLIB.Service({
				ros: ros,
				name: '/bucket/rearm',
				serviceType: 'std_srvs/srv/Trigger'
			});
			operatorControl = new ROSLIB.Topic({
				ros: ros,
				name: '/bucket/operator_override',
				messageType: 'std_msgs/Bool'
			});

			jointState.subscribe((message: JointStateType) => {
				const jawIndex = message.name.indexOf('bucket_jaw_joint');
				const tiltIndex = message.name.indexOf('bucket_tilt_joint');
				const liftIndex = message.name.indexOf('bucket_lift_joint');
				if (jawIndex !== -1 && tiltIndex !== -1 && liftIndex !== -1) {
					currentAngles.jaw = message.position[jawIndex] * (180 / Math.PI);
					currentAngles.tilt = message.position[tiltIndex] * (180 / Math.PI);
					currentAngles.lift = message.position[liftIndex] * (180 / Math.PI);
				}
			});
			operatorControl.subscribe((message: BoolType) => {
				isOperatorControl = message.data;
			});
		} else {
			jointState = null;
			action = null;
			rearm = null;
			operatorControl = null;
		}
	});

	const setTarget = async (angles: Angles) => {
		if (isOperatorControl) return;
		targetAngles = angles;
		if (action) {
			const goal: ActionGoalType = {
				trajectory: {
					joint_names: ['bucket_jaw_joint', 'bucket_tilt_joint', 'bucket_lift_joint'],
					points: [
						{
							positions: [
								angles.jaw * (Math.PI / 180),
								angles.tilt * (Math.PI / 180),
								angles.lift * (Math.PI / 180)
							],
							time_from_start: { sec: 10, nanosec: 0 }
						}
					]
				}
			};

			if (!action) {
				return;
			}
			errorMessage = null;
			lastGoalId =
				action.sendGoal(
					goal,
					(result) => {
						if (result.error_code !== 0) {
							errorMessage = `Goal failed (${result.error_code}): ${result.error_string}`;
						}
						lastGoalId = null;
					},
					() => {
						// ignore feedback for now
					},
					(error) => {
						errorMessage = `Goal failed: ${error}`;
						lastGoalId = null;
					} // goal rejected, aborted or cancelled
				) ?? null;
			if (lastGoalId === null) errorMessage = 'Failed to send goal';
		}
	};

	const sendRearm = () => {
		if (rearm) {
			rearm.callService({}, (response) => {
				console.log('Rearm response:', response);
			});
		}
	};

	const cancelGoal = () => {
		if (action && lastGoalId !== null) {
			action.cancelGoal(lastGoalId);
			console.log('Goal cancelled');
		}
	};
</script>

<div class="flex h-full w-full flex-col">
	<div class="h-100 flex w-full flex-1 gap-2 overflow-y-hidden">
		<div class="relative min-w-0 flex-1">
			{#if errorMessage}
				<kbd class="absolute left-2 top-1 text-sm text-red-600">
					{errorMessage}
				</kbd>
			{/if}
			<div class="absolute inset-0">
				<CanvasWrapper ondraw={drawMain} worldWidth={WIDTH} worldHeight={HEIGHT} />
			</div>
			<kbd class="absolute bottom-1 left-2 text-sm">
				Current positions:<br />Jaw: {currentAngles.jaw.toFixed(1)}, Tilt: {currentAngles.tilt.toFixed(
					1
				)}, Lift: {currentAngles.lift.toFixed(1)}
			</kbd>
		</div>

		<div
			class={`presets flex w-64 shrink-0 flex-col gap-2 overflow-y-auto rounded-2xl border pb-2 ${isOperatorControl ? 'opacity-50' : ''}`}
		>
			{#each presets as preset}
				<button
					class="preset block pt-2 text-left"
					onclick={() => setTarget({ jaw: preset.jaw, tilt: preset.tilt, lift: preset.lift })}
				>
					<span class="relative flex aspect-[3/2] w-full flex-col">
						<p class="text-center">{preset.name}</p>
						<span class="absolute inset-0">
							<CanvasWrapper
								ondraw={(c) => drawCentered(c, toVirtual(preset), 1)}
								worldWidth={WIDTH}
								worldHeight={HEIGHT}
							/>
						</span>
						<p class="mt-auto text-center text-sm opacity-65">
							Jaw: {preset.jaw.toFixed(1)}, Tilt: {preset.tilt.toFixed(1)}, Lift: {preset.lift.toFixed(
								1
							)}
						</p>
					</span>
				</button>
			{/each}
		</div>
	</div>
	<div class="h-[60px]">
		{#if isOperatorControl}
			<div class="flex h-full w-full items-center justify-center gap-2">
				<p class="text-red-600">Operator Control Active</p>
				<Button variant="destructive" onclick={sendRearm}>Rearm</Button>
			</div>
		{:else}
			<div class="flex h-full w-full flex-row items-center justify-center gap-2">
				<Label><kbd>Jaw:</kbd></Label>
				<Input type="number" bind:value={inputAngles.jaw} max={30} min={0} />
				<Label><kbd>Tilt:</kbd></Label>
				<Input type="number" bind:value={inputAngles.tilt} max={30} min={-45} />
				<Label><kbd>Lift:</kbd></Label>
				<Input type="number" bind:value={inputAngles.lift} max={45} min={0} />
				<Button onclick={() => setTarget(inputAngles)}>Send Target</Button>
				<Button disabled={lastGoalId === null} variant="destructive" onclick={() => cancelGoal()}>
					Cancel Goal
				</Button>
			</div>
		{/if}
	</div>
</div>

<style>
	.preset:not(:first-child) {
		border-top: 1px solid;
		border-top-color: inherit;
	}

	/* make scroll bar nice in presets */
	.presets::-webkit-scrollbar {
		width: 14px;
	}
	.presets::-webkit-scrollbar-button {
		display: none; /* removes the up/down arrows */
	}
	.presets::-webkit-scrollbar-track {
		background: transparent;
		margin-block: 12px; /* keeps the ends clear of the rounded container corners */
	}
	.presets::-webkit-scrollbar-thumb {
		background: #888;
		border-radius: 9999px; /* fully circular ends */
		border: 4px solid transparent; /* with background-clip, makes the pill 4px wide, inset from the edge */
		background-clip: content-box;
	}
	.presets::-webkit-scrollbar-thumb:hover {
		background: #bbb;
		background-clip: content-box;
	}

	/* Firefox has no pseudo-elements: thin and coloured is the most it supports */
	@supports not selector(::-webkit-scrollbar) {
		.presets {
			scrollbar-width: thin;
			scrollbar-color: #888 transparent;
		}
	}
</style>
