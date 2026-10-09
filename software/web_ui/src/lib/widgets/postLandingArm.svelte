<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Post Landing Arm';
	export const description =
		'Teleoperate the post-landing arm: Cartesian twist and joint jogging via MoveIt Servo, direct wrist and gripper control, and homing.';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				linearSpeed: {
					type: 'number',
					description: 'Cartesian jog speed in m/s (twist mode)',
					value: '0.1'
				},
				jointSpeed: {
					type: 'number',
					description: 'Joint jog speed in rad/s (joint mode)',
					value: '0.5'
				},
				gripperStep: {
					type: 'number',
					description: 'Gripper jaw step in mm',
					value: '3'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount, untrack } from 'svelte';
	import { toast } from 'svelte-sonner';
	import Button from '$lib/components/ui/button/button.svelte';

	// Arm model (mirrors payloads/scripts/keyboard_control.py)
	const SERVO_JOINTS = ['shoulder_pan', 'shoulder_tilt', 'elbow'];
	const WRIST_JOINTS = ['wrist_pitch', 'wrist_roll'];
	const ALL_JOINTS = [...SERVO_JOINTS, ...WRIST_JOINTS];
	const WRIST_LIMITS: Record<string, [number, number]> = {
		wrist_pitch: [-1.57, 1.57],
		wrist_roll: [-6.28318, 6.28318]
	};
	const GRIPPER_OPEN = 0.015; // m, per-finger travel (jaw gap is 2x)
	const GRIPPER_JOINT = 'left_finger_joint';
	const PUBLISH_PERIOD = 50; // ms
	const HOME_SERVICE_TIMEOUT = 30; // seconds; arm_home blocks for the whole homing move
	const WRIST_DURATION = { sec: 0, nanosec: 2 * PUBLISH_PERIOD * 1_000_000 };
	const GRIPPER_DURATION = { sec: 0, nanosec: 300_000_000 };

	// moveit_msgs/srv/ServoCommandType constants (moveit_msgs 2.6.0: JOINT_JOG=0, TWIST=1, POSE=2)
	const SERVO_COMMAND_JOINT_JOG = 0;
	const SERVO_COMMAND_TWIST = 1;

	// ---------------- UI state ----------------
	let mode = $state<'twist' | 'joint'>('twist');
	let frameId = $state('plate'); // 'plate' = world frame, 'forearm_tip' = end effector
	let selectedJoint = $state<string>(SERVO_JOINTS[0]);
	let held = $state<Record<string, boolean>>({}); // 'x+'|'x-'|'y+'|'y-'|'z+'|'z-'|'j+'|'j-'
	let jointPositions = $state<Record<string, number>>({});
	let gripperTarget = $state(0);
	let srvStatus = $state('ROS disconnected');
	let homeBusy = $state(false);

	// ---------------- ROS handles ----------------
	let twistPub: ROSLIB.Topic | null = null;
	let jointPub: ROSLIB.Topic | null = null;
	let wristPub: ROSLIB.Topic | null = null;
	let gripperPub: ROSLIB.Topic | null = null;
	let jointStateSub: ROSLIB.Topic | null = null;
	let switchSrv: ROSLIB.Service | null = null;
	let homeSrv: ROSLIB.Service | null = null;

	// Wrist position target, seeded from /joint_states then integrated locally
	let wristTarget: Record<string, number> | null = null;

	const anyHeld = () => Object.values(held).some(Boolean);

	const publishZeroTwist = () => {
		// Actively stop the servo jog instead of waiting for its command timeout
		twistPub?.publish({
			header: {},
			twist: {
				linear: { x: 0, y: 0, z: 0 },
				angular: { x: 0, y: 0, z: 0 }
			}
		});
	};

	const holdHandlers = (key: string) => ({
		onpointerdown: (e: PointerEvent) => {
			if (e.button === 0) held[key] = true;
		},
		onpointerup: () => {
			held[key] = false;
			if (!anyHeld() && mode === 'twist') publishZeroTwist();
		},
		onpointerleave: () => {
			held[key] = false;
			if (!anyHeld() && mode === 'twist') publishZeroTwist();
		},
		onpointercancel: () => {
			held[key] = false;
			if (!anyHeld() && mode === 'twist') publishZeroTwist();
		}
	});

	// ---------------- Command helpers ----------------
	const publishWrist = (joint: string, velocity: number) => {
		if (!wristPub) return;

		// Seed the target from the latest joint states on first use
		if (wristTarget === null) {
			if (WRIST_JOINTS.every((j) => jointPositions[j] !== undefined)) {
				wristTarget = Object.fromEntries(WRIST_JOINTS.map((j) => [j, jointPositions[j]]));
			} else {
				srvStatus = 'Waiting for /joint_states...';
				return;
			}
		}

		const [lo, hi] = WRIST_LIMITS[joint];
		const dt = PUBLISH_PERIOD / 1000;
		wristTarget = {
			...wristTarget,
			[joint]: Math.max(lo, Math.min(hi, wristTarget[joint] + velocity * dt))
		};

		wristPub.publish({
			joint_names: [...WRIST_JOINTS],
			points: [
				{
					positions: [wristTarget.wrist_pitch, wristTarget.wrist_roll],
					velocities: [0, 0],
					time_from_start: WRIST_DURATION
				}
			]
		});
	};

	const commandGripper = (target: number) => {
		if (!gripperPub) {
			srvStatus = 'ROS disconnected';
			return;
		}
		gripperTarget = Math.max(0, Math.min(GRIPPER_OPEN, target));
		gripperPub.publish({
			joint_names: [GRIPPER_JOINT],
			points: [
				{ positions: [gripperTarget], velocities: [0], time_from_start: GRIPPER_DURATION }
			]
		});
	};

	const gripperStepM = () => (Number(settings.groups.General.gripperStep.value) || 0) / 1000;

	// Published at a fixed rate while a jog button is held, like keyboard_control.py
	const tick = () => {
		if (!anyHeld()) return;

		if (mode === 'twist') {
			if (!twistPub) return;
			const speed = Number(settings.groups.General.linearSpeed.value) || 0;
			const linear = { x: 0, y: 0, z: 0 };
			if (held['x+']) linear.x = speed;
			if (held['x-']) linear.x = -speed;
			if (held['y+']) linear.y = speed;
			if (held['y-']) linear.y = -speed;
			if (held['z+']) linear.z = speed;
			if (held['z-']) linear.z = -speed;
			twistPub.publish({
				header: { frame_id: frameId },
				twist: { linear, angular: { x: 0, y: 0, z: 0 } }
			});
		} else {
			const dir = (held['j+'] ? 1 : 0) - (held['j-'] ? 1 : 0);
			if (!dir) return;
			const speed = Number(settings.groups.General.jointSpeed.value) || 0;

			// Servo joints jog through MoveIt Servo; wrist joints bypass it
			if (SERVO_JOINTS.includes(selectedJoint)) {
				jointPub?.publish({
					joint_names: [selectedJoint],
					velocities: [dir * speed],
					duration: 0
				});
			} else if (WRIST_JOINTS.includes(selectedJoint)) {
				publishWrist(selectedJoint, dir * speed);
			}
		}
	};

	// ---------------- Services ----------------
	const callSwitchService = (newMode: 'twist' | 'joint', onSuccess?: () => void) => {
		if (!switchSrv) {
			onSuccess?.();
			return;
		}
		switchSrv.callService(
			{
				command_type: newMode === 'twist' ? SERVO_COMMAND_TWIST : SERVO_COMMAND_JOINT_JOG
			},
			(resp: any) => {
				if (resp?.success) {
					onSuccess?.();
				} else {
					srvStatus = resp?.message ?? 'Failed to switch servo command type';
					toast.error(srvStatus);
				}
			},
			(err: any) => {
				srvStatus = `Mode switch failed: ${err?.toString?.() ?? err}`;
				toast.error(srvStatus);
			}
		);
	};

	const setMode = (newMode: 'twist' | 'joint') => {
		if (newMode === mode) return;

		if (!switchSrv) {
			mode = newMode;
			held = {};
			return;
		}

		callSwitchService(newMode, () => {
			mode = newMode;
			held = {};
			srvStatus = `${newMode === 'twist' ? 'Twist' : 'Joint'} mode`;
		});
	};

	const homeArm = () => {
		if (!homeSrv) {
			srvStatus = 'Home service not ready';
			return;
		}
		homeBusy = true;
		homeSrv.callService(
			{},
			(resp: any) => {
				homeBusy = false;
				srvStatus = resp?.message ?? (resp?.success ? 'Homing complete' : 'Home failed');
				if (resp?.success) toast.success(srvStatus);
				else toast.error(srvStatus);
			},
			(err: any) => {
				homeBusy = false;
				srvStatus = `Home failed: ${err?.toString?.() ?? err}`;
				toast.error(srvStatus);
			},
			// arm_home blocks for the whole homing move (~10s); rosbridge's
			// default service timeout of 5s would flag this as failed mid-move
			HOME_SERVICE_TIMEOUT
		);
	};

	// ---------------- ROS connection ----------------
	// Only getRosConnection() is tracked. ROSLIB mutates the $state-proxied Ros
	// object while creating topics/calling services, which would otherwise
	// re-trigger this effect in an infinite loop.
	$effect(() => {
		const ros = getRosConnection();
		if (!ros) {
			twistPub = null;
			jointPub = null;
			wristPub = null;
			gripperPub = null;
			switchSrv = null;
			homeSrv = null;
			jointStateSub?.unsubscribe();
			jointStateSub = null;
			held = {};
			srvStatus = 'ROS disconnected';
			return;
		}

		untrack(() => {
			twistPub = new ROSLIB.Topic({
			ros,
			name: '/servo_node/delta_twist_cmds',
			messageType: 'geometry_msgs/TwistStamped'
		});
		jointPub = new ROSLIB.Topic({
			ros,
			name: '/servo_node/delta_joint_cmds',
			messageType: 'control_msgs/JointJog'
		});
		wristPub = new ROSLIB.Topic({
			ros,
			name: '/wrist_controller/joint_trajectory',
			messageType: 'trajectory_msgs/JointTrajectory'
		});
		gripperPub = new ROSLIB.Topic({
			ros,
			name: '/gripper_controller/joint_trajectory',
			messageType: 'trajectory_msgs/JointTrajectory'
		});

		switchSrv = new ROSLIB.Service({
			ros,
			name: '/servo_node/switch_command_type',
			serviceType: 'moveit_msgs/srv/ServoCommandType'
		});
		homeSrv = new ROSLIB.Service({
			ros,
			name: '/arm_home/move',
			serviceType: 'std_srvs/srv/Trigger'
		});

		jointStateSub = new ROSLIB.Topic({
			ros,
			name: '/joint_states',
			messageType: 'sensor_msgs/JointState'
		});
		jointStateSub.subscribe((msg: any) => {
			msg.name?.forEach((joint: string, i: number) => {
				if (msg.position?.[i] !== undefined) jointPositions[joint] = msg.position[i];
			});
		});

			srvStatus = 'ROS connected';

			// MoveIt Servo ignores commands that don't match its command type, so sync
			// it with the widget's mode on connect (mirrors keyboard_control.py startup)
			callSwitchService(mode, () => {
				srvStatus = `Servo synced (${mode} mode)`;
			});
		});
	});

	onMount(() => {
		const intervalHandle = setInterval(tick, PUBLISH_PERIOD);
		return () => {
			clearInterval(intervalHandle);
			jointStateSub?.unsubscribe();
		};
	});
</script>

<svelte:window onblur={() => (held = {})} />

<div class="flex h-full w-full flex-col gap-2 overflow-auto p-1 text-sm">
	<!-- Mode toggle + frame / joint selection -->
	<div class="flex flex-wrap items-center gap-1">
		<Button
			size="sm"
			variant={mode === 'twist' ? 'default' : 'outline'}
			onclick={() => setMode('twist')}
		>
			Twist
		</Button>
		<Button
			size="sm"
			variant={mode === 'joint' ? 'default' : 'outline'}
			onclick={() => setMode('joint')}
		>
			Joint
		</Button>

		{#if mode === 'twist'}
			<select
				bind:value={frameId}
				class="h-8 rounded-md border bg-background px-2 text-xs hover:bg-accent"
			>
				<option value="plate">World Frame</option>
				<option value="forearm_tip">End Effector</option>
			</select>
		{:else}
			<select
				bind:value={selectedJoint}
				class="h-8 rounded-md border bg-background px-2 text-xs hover:bg-accent"
			>
				{#each ALL_JOINTS as joint}
					<option value={joint}>{joint}</option>
				{/each}
			</select>
		{/if}
	</div>

	{#if mode === 'twist'}
		<!-- Cartesian jog: columns are Y, Z, X -->
		<div class="grid grid-cols-3 gap-1">
			<Button size="sm" variant="outline" {...holdHandlers('y+')}>Y +</Button>
			<Button size="sm" variant="outline" {...holdHandlers('z+')}>Z +</Button>
			<Button size="sm" variant="outline" {...holdHandlers('x+')}>X +</Button>
			<Button size="sm" variant="outline" {...holdHandlers('y-')}>Y -</Button>
			<Button size="sm" variant="outline" {...holdHandlers('z-')}>Z -</Button>
			<Button size="sm" variant="outline" {...holdHandlers('x-')}>X -</Button>
		</div>
	{:else}
		<div class="grid grid-cols-2 gap-1">
			<Button size="sm" variant="outline" {...holdHandlers('j-')}>Jog -</Button>
			<Button size="sm" variant="outline" {...holdHandlers('j+')}>Jog +</Button>
		</div>
	{/if}

	<!-- Gripper -->
	<div class="grid grid-cols-4 gap-1">
		<Button size="sm" variant="outline" onclick={() => commandGripper(GRIPPER_OPEN)}
			>Open</Button
		>
		<Button size="sm" variant="outline" onclick={() => commandGripper(0)}>Close</Button>
		<Button
			size="sm"
			variant="outline"
			onclick={() => commandGripper(gripperTarget + gripperStepM())}>Step +</Button
		>
		<Button
			size="sm"
			variant="outline"
			onclick={() => commandGripper(gripperTarget - gripperStepM())}>Step -</Button
		>
	</div>
	<p class="text-xs opacity-70">
		Gripper jaw: <span class="font-mono">{(gripperTarget * 2000).toFixed(0)} mm</span>
	</p>

	<!-- Homing -->
	<Button size="sm" class="w-full font-bold" disabled={homeBusy} onclick={homeArm}>
		{homeBusy ? 'HOMING...' : 'Home Arm'}
	</Button>

	<!-- Status + joint readout -->
	<p class="text-xs opacity-70">
		Status: <span class="font-mono">{srvStatus}</span>
	</p>
	<div class="min-h-0 flex-1 font-mono text-xs opacity-70">
		{#each ALL_JOINTS as joint}
			<p class="whitespace-nowrap">
				{joint}:
				{jointPositions[joint] !== undefined
					? `${jointPositions[joint].toFixed(2)} rad`
					: '—'}
			</p>
		{/each}
	</div>
</div>
