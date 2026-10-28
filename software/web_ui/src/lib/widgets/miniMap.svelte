<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Mini Map';
	// These properties are optional
	export const description = 'Display a mini map of the arena for Lunabotics';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});

	export interface ArenaLayout {
		frame: string;
		zones: Zone[];
		fiducials: Fiducials;
	}

	export interface Zone {
		name: string;
		// center x and y
		x: number;
		y: number;
		width: number;
		height: number;
		color: [number, number, number];
		// only draw the wall for the edges specified in this array
		edges: Array<'N' | 'E' | 'S' | 'W'>;
	}

	export interface Fiducials {
		dictionary_id: number;
		marker_length_m: number;
		spacing_m: number;
		spacing_tolerance_m: number;
		markers: Marker[];
	}

	export interface Marker {
		id: number;
		x: number;
		y: number;
		z: number;
	}

	const rawArenaLayout: Record<string, { default: ArenaLayout }> = import.meta.glob(
		'../../../../**/arena_layout.json',
		{ eager: true }
	);
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import type { OdometryType } from '$lib/scripts/rosTypes';

	const SCALE = 40;
	const TEXT_OFFSET = 2;

	let { class: clazz }: { class?: string } = $props();

	let odometryTopic: ROSLIB.Topic<OdometryType> | null = null;

	let pose = $state<OdometryType['pose']['pose']>({
		orientation: { x: 1, y: 2, z: 2, w: 1 },
		position: { x: 0, y: 0, z: 0 }
	});
	let invalidLayout = $state(false);
	let arenaLayout = $derived<ArenaLayout>(Object.values(rawArenaLayout)[0].default);
	let zones = $derived.by(() => {
		if (invalidLayout) return [];
		return arenaLayout.zones;
	});
	let markers = $derived.by(() => {
		if (invalidLayout) return [];
		return arenaLayout.fiducials.markers;
	});

	let totalSize = $derived.by(() => {
		if (invalidLayout) return { xMax: 0, xMin: 0, yMax: 0, yMin: 0 };
		const xMin = Math.min(...arenaLayout.zones.map((zone) => zone.x - zone.width / 2));
		const xMax = Math.max(...arenaLayout.zones.map((zone) => zone.x + zone.width / 2));
		const yMin = Math.min(...arenaLayout.zones.map((zone) => zone.y - zone.height / 2));
		const yMax = Math.max(...arenaLayout.zones.map((zone) => zone.y + zone.height / 2));
		return { xMax, xMin, yMax, yMin };
	});
	// World y points up but SVG y points down, so the top of the view is -yMax.
	let viewBox = $derived(
		[
			totalSize.xMin * SCALE,
			-totalSize.yMax * SCALE,
			Math.max(1, (totalSize.xMax - totalSize.xMin) * SCALE),
			Math.max(1, (totalSize.yMax - totalSize.yMin) * SCALE)
		].join(' ')
	);

	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			odometryTopic = new ROSLIB.Topic({
				ros,
				name: '/odometry/filtered',
				messageType: 'nav_msgs/Odometry'
			});

			odometryTopic.subscribe((msg) => {
				pose = msg.pose.pose;
			});
		} else {
			odometryTopic = null;
		}
	});

	onMount(() => {
		// ensure we have the arena layout file
		if (!arenaLayout || Object.values(arenaLayout).length === 0) {
			invalidLayout = true;
		}
	});
</script>

<svg {viewBox} class="h-full w-full {clazz}">
	{#each zones as zone}
		{@const fillColor = `rgba(${zone.color[0] * 255}, ${zone.color[1] * 255}, ${zone.color[2] * 255}, 0.5)`}
		{@const strokeColor = `rgb(${zone.color[0] * 255}, ${zone.color[1] * 255}, ${zone.color[2] * 255})`}
		<rect
			x={(zone.x - zone.width / 2) * SCALE}
			y={-(zone.y + zone.height / 2) * SCALE}
			width={zone.width * SCALE}
			height={zone.height * SCALE}
			fill={fillColor}
			stroke={strokeColor}
			stroke-width="1"
		/>
		<text
			x={(zone.x - zone.width / 2) * SCALE + TEXT_OFFSET}
			y={-(zone.y + zone.height / 2) * SCALE + TEXT_OFFSET}
			font-size="8"
			text-anchor="start"
			dominant-baseline="hanging"
			fill={fillColor}
		>
			{zone.name}
		</text>
		{#each zone.edges as edge}
			{@const signX1 = edge === 'E' ? 1 : -1}
			{@const signY1 = edge === 'S' ? -1 : 1}
			{@const signX2 = edge === 'W' ? -1 : 1}
			{@const signY2 = edge === 'N' ? 1 : -1}
			<line
				x1={(zone.x + (signX1 * zone.width) / 2) * SCALE}
				y1={-(zone.y + (signY1 * zone.height) / 2) * SCALE}
				x2={(zone.x + (signX2 * zone.width) / 2) * SCALE}
				y2={-(zone.y + (signY2 * zone.height) / 2) * SCALE}
				stroke={strokeColor}
				stroke-width="1"
			/>
		{/each}
	{/each}
	{#each markers as marker}
		<circle
			cx={marker.x * SCALE}
			cy={marker.y * SCALE}
			r={arenaLayout.fiducials.marker_length_m / 2}
			fill="none"
			stroke="black"
			stroke-width="1"
		/>
	{/each}

	<!-- Current position of the rover -->
	<g
		transform={`translate(${pose.position.x * SCALE}, ${pose.position.y * SCALE})`}
		style:transition="transform 0.1s ease-in-out"
	>
		<circle r="5" fill="red" />
		<line
			x1="0"
			y1="0"
			x2={10 * (2 * Math.atan2(pose.orientation.z, pose.orientation.w))}
			y2={10 * (2 * Math.asin(pose.orientation.y))}
			stroke="black"
			stroke-width="1"
		/>
	</g>
</svg>
