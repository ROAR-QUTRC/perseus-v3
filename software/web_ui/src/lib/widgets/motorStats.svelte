<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Motor Stats';
	export const description = 'View live stats for each of the motors.';
	export const isRosDependent = true;
	export const group = 'CAN Bus';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});

	export interface MotorStatsType {
		targetRPM: number;
		currentRPM: number;
		current: number;
		temperature: number;
	}
</script>

<script lang="ts">
	import * as ROSLIB from 'roslib';
	import Gauge from './motorStats/gauge.svelte';
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';

	const WHEEL_DIAMETER = 0.26; // meters taken from CAD
	const MAX_LINEAR_VELOCITY = 1.7; // m/s taken from software/ros_ws/src/perseus/config/perseus_controllers.yaml

	const WHEEL_IDS = ['front_left', 'front_right', 'rear_left', 'rear_right'] as const;
	const TARGET_PREFIX = 'command_interface.wheel_pid_controller/' as const;
	const VALUE_PREFIX = 'state_interface.wheel_pid_controller/' as const;
	const VELOCITY_SUFFIX = '_wheel_joint/velocity' as const;
	const CURRENT_SUFFIX = '_wheel_joint/current' as const;
	const CURRENT_PREFIX = 'state_interface.' as const;

	type VelocityTargetIdType =
		`${typeof TARGET_PREFIX}${(typeof WHEEL_IDS)[number]}${typeof VELOCITY_SUFFIX}`;
	type VelocityValueIdType =
		`${typeof VALUE_PREFIX}${(typeof WHEEL_IDS)[number]}${typeof VELOCITY_SUFFIX}`;
	type CurrentValueIdType =
		`${typeof CURRENT_PREFIX}${(typeof WHEEL_IDS)[number]}${typeof CURRENT_SUFFIX}`;
	interface WheelTopicType {
		header: object; // ignored here
		statistics: Array<{
			// There are other name values but we only care about the velocity and current ones for this widget
			name: VelocityTargetIdType | VelocityValueIdType | CurrentValueIdType;
			value: number;
		}>;
	}
	interface WheelDataType {
		value: number;
		target: number;
		current: number;
	}

	let clientWidth = $state(0);
	let clientHeight = $state(0);

	let wheelTopic: ROSLIB.Topic<WheelTopicType> | null = null;
	let wheelData = $state<Record<(typeof WHEEL_IDS)[number], WheelDataType>>({
		front_left: { value: 0, target: 0, current: 0 },
		front_right: { value: 0, target: 0, current: 0 },
		rear_left: { value: 0, target: 0, current: 0 },
		rear_right: { value: 0, target: 0, current: 0 }
	});

	$effect(() => {
		const ros = getRosConnection();

		if (ros) {
			wheelTopic = new ROSLIB.Topic({
				ros,
				name: '/controller_manager/introspection_data/full',
				messageType: 'pal_statistics_msgs/msg/Statistics'
			});

			wheelTopic.subscribe((message: WheelTopicType) => {
				message.statistics.forEach((stat) => {
					let propertyName: keyof WheelDataType | null = null;
					let prefix: string | null = null;
					if (stat.name.startsWith(TARGET_PREFIX)) {
						propertyName = 'target';
						prefix = TARGET_PREFIX;
					} else if (stat.name.startsWith(VALUE_PREFIX)) {
						propertyName = 'value';
						prefix = VALUE_PREFIX;
					} else if (stat.name.startsWith(CURRENT_PREFIX)) {
						propertyName = 'current';
						prefix = CURRENT_PREFIX;
					} else {
						return;
					}
					const wheelId = stat.name.slice(
						prefix.length,
						stat.name.length -
							(propertyName === 'current' ? CURRENT_SUFFIX.length : VELOCITY_SUFFIX.length)
					) as (typeof WHEEL_IDS)[number];
					if (!WHEEL_IDS.includes(wheelId as (typeof WHEEL_IDS)[number])) {
						return;
					}

					wheelData[wheelId][propertyName] = stat.value;
				});
			});
		} else {
			wheelTopic = null;
		}
	});

	// test n layouts to find the best one
	const layout = $derived.by(() => {
		const count = Object.keys(wheelData).length;
		if (!count || clientWidth <= 0 || clientHeight <= 0) return { size: 0, cols: 1 };

		let best = { size: 0, cols: 1 };
		for (let cols = 1; cols <= count; cols++) {
			const rows = Math.ceil(count / cols);
			const newSize = Math.min(
				(clientWidth - (cols - 1) * 2) / cols,
				(clientHeight - (rows - 1) * 2) / rows
			);
			if (newSize > best.size) best = { size: newSize, cols };
		}
		return { size: Math.floor(best.size), cols: best.cols };
	});

	const angularToLinearVelocity = (angularVelocity: number) => {
		return angularVelocity * (WHEEL_DIAMETER / 2);
	};
</script>

<div
	class="grid h-full w-full place-content-center overflow-hidden"
	style:grid-template-columns="repeat({layout.cols}, {layout.size}px)"
	style:grid-auto-rows="{layout.size}px"
	style:gap="{2}px"
	bind:clientWidth
	bind:clientHeight
>
	{#each Object.entries(wheelData) as [key, motor]}
		<Gauge
			value={angularToLinearVelocity(motor.value)}
			target={angularToLinearVelocity(motor.target)}
			secondaryValue={motor.current}
			secondaryLabel="Amps"
			max={MAX_LINEAR_VELOCITY}
			label={key}
		/>
	{/each}
</div>
