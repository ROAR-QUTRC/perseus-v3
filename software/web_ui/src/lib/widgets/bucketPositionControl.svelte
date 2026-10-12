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
	import { onMount } from 'svelte';

	let presets = $state<
		Array<{
			name: string;
			canvas: CanvasApi | null;
			jaw: number;
			tilt: number;
			lift: number;
		}>
	>([
		{ name: 'Home', canvas: null, jaw: 0, tilt: 0, lift: 0 },
		{ name: 'Preset 1', canvas: null, jaw: 10, tilt: 20, lift: 30 },
		{ name: 'Preset 2', canvas: null, jaw: -10, tilt: -20, lift: -30 }
	]);

	const angleOffset = { jaw: 180, tilt: 255, lift: 197 };

	let canvas = $state<CanvasApi | null>(null);
	let currentAngles = $state({ jaw: 0, tilt: 0, lift: 0 });
	let targetAngles = $state({ jaw: 0, tilt: 0, lift: 0 });

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

	const HALF_BASE_WIDTH = 120;
	const ACTUATOR_SLEEVE_LENGTH = 350;
	const ACTUATOR_ARM_LENGTH = 286;
	const CENTRAL_MEMBER_LENGTH = 750;
	const LIFT_ACTUATOR_OFFSET = 117;
	const TILT_ACTUATOR_OFFSET = 250;
	const BUCKET_BASE = 140;
	const BUCKET_DEPTH = 100;
	const X_OFFSET = 400;

	let virtualAngles = $derived({
		jaw: -(currentAngles.jaw + angleOffset.jaw),
		tilt: currentAngles.tilt + angleOffset.tilt,
		lift: currentAngles.lift + angleOffset.lift
	});

	let virtualTargets = $derived({
		jaw: -(targetAngles.jaw + angleOffset.jaw),
		tilt: targetAngles.tilt + angleOffset.tilt,
		lift: targetAngles.lift + angleOffset.lift
	});

	const drawBucket = (
		canvas: CanvasApi,
		angles: { jaw: number; tilt: number; lift: number },
		opacity: number
	) => {
		console.log('drawBucket', angles, opacity);
		canvas.setOpacity(opacity);
		// base plate center is reference point
		const baseCenter = { x: X_OFFSET, y: 0 };
		const baseTop = { x: X_OFFSET, y: HALF_BASE_WIDTH };
		const baseBottom = { x: X_OFFSET, y: -HALF_BASE_WIDTH };
		canvas.drawLine(baseCenter.x, baseCenter.y, baseTop.x, baseTop.y, 'grey', 4);
		canvas.drawLine(baseCenter.x, baseCenter.y, baseBottom.x, baseBottom.y, 'grey', 4);
		// center member
		canvas.drawLineAngle(
			X_OFFSET,
			HALF_BASE_WIDTH,
			angles.lift,
			CENTRAL_MEMBER_LENGTH,
			'lightgrey',
			4
		);
		const memberEndPoint = polarToCartesian(
			CENTRAL_MEMBER_LENGTH,
			angles.lift,
			baseTop.x,
			baseTop.y
		);
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
		canvas.drawLine(bucketTop.x, bucketTop.y, jawRightBase.x, jawRightBase.y, 'dimgrey', 4);
		canvas.drawLine(jawRightBase.x, jawRightBase.y, jawLeftBase.x, jawLeftBase.y, 'dimgrey', 4);
		canvas.drawLine(jawLeftBase.x, jawLeftBase.y, bucketTop.x, bucketTop.y, 'dimgrey', 4);

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

	$effect(() => {
		if (!canvas) return;
		canvas.clear();
		drawBucket(canvas, virtualTargets, 0.5);
		drawBucket(canvas, virtualAngles, 1);
	});

	onMount(() => {
		let counter = 0;
		setInterval(() => {
			// sine the angles
			currentAngles.jaw = Math.sin(counter / 50) * 20;
			currentAngles.tilt = Math.sin(counter / 50) * 20;
			currentAngles.lift = Math.sin(counter / 50) * 20;
			counter++;
		}, 80);
	});
</script>

<div class="flex flex-row items-center gap-2">
	<div class="w-4/5 border">
		<CanvasWrapper bind:api={canvas} worldWidth={600} worldHeight={600} />
		<p>
			Current Angles:<br />Jaw: {currentAngles.jaw}, Tilt: {currentAngles.tilt}, Lift: {currentAngles.lift}
		</p>
	</div>
	<div class="flex h-full flex-col overflow-scroll border">
		{#each presets as preset}
			<CanvasWrapper
				ondraw={(c) => drawBucket(c, { jaw: preset.jaw, tilt: preset.tilt, lift: preset.lift }, 1)}
				worldWidth={200}
				worldHeight={100}
			/>
			<p>{preset.name}</p>
		{/each}
	</div>
</div>
