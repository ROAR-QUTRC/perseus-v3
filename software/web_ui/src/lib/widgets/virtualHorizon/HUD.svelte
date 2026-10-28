<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import * as ROSLIB from 'roslib';
	import type { ClassNameValue } from 'cn';
	import type { OdometryType } from '$lib/scripts/rosTypes';

	const CROSSHAIR_SIZE = 20;
	const YAW_SIZE = 20;
	const LINE_THICKNESS = 2; // px
	const RAD_TO_DEG = 180 / Math.PI;

	// roll  -> x -> forward
	// pitch -> y -> left
	// yaw   -> z -> up
	let { color = 'text-primary', class: clazz }: { color?: string; class?: ClassNameValue } =
		$props();
	const uid = $props.id();

	let roll = $state(0);
	let pitch = $state(0);
	let yaw = $state(0);

	let odomTopic: ROSLIB.Topic<OdometryType> | null = null;

	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			odomTopic = new ROSLIB.Topic({
				ros,
				name: '/odometry/filtered',
				messageType: 'nav_msgs/Odometry'
			});

			odomTopic.subscribe((msg) => {
				yaw = msg.pose.pose.orientation.z;
				pitch = msg.pose.pose.orientation.y;
				roll = msg.pose.pose.orientation.x;
			});
		}
	});

	let width = $state(0);
	let height = $state(0);
	let halfWidth = $derived(width / 2);
	let halfHeight = $derived(height / 2);

	let pixelPerDegVertical = $derived<number>(width / 360);
	let pixelPerDegHorizontal = $derived<number>(height / 180);
	let yawRange = $derived.by(() => {
		const yawDeg = yaw * RAD_TO_DEG;
		const roundDeg = Math.round(yawDeg / 10) * 10;

		const out = [];
		for (let i = roundDeg - 360; i < roundDeg + 360; i += 10) {
			out.push(i);
		}
		return out;
	});

	let pitchLevels = Array.from({ length: 37 }, (_, i) => (i - 18) * 10); // -180 to +180 in steps of 10
</script>

<svg
	viewBox="{-halfWidth} {-halfHeight} {width} {height}"
	class="h-full w-full {color} {clazz}"
	bind:clientWidth={width}
	bind:clientHeight={height}
>
	<!-- Gradients -->
	<defs>
		<!-- Fades pitch bars toward the top and bottom edges -->
		<linearGradient id="{uid}-fade-y-gradient" x1="0" y1="0" x2="0" y2="1">
			<stop offset="0" stop-color="white" stop-opacity="0" />
			<stop offset="0.2" stop-color="white" stop-opacity="1" />
			<stop offset="0.8" stop-color="white" stop-opacity="1" />
			<stop offset="1" stop-color="white" stop-opacity="0" />
		</linearGradient>
		<mask
			id="{uid}-fade-y"
			maskUnits="userSpaceOnUse"
			x={-halfWidth}
			y={-halfHeight}
			{width}
			{height}
		>
			<rect x={-halfWidth} y={-halfHeight} {width} {height} fill="url(#{uid}-fade-y-gradient)" />
		</mask>

		<!-- Fades the yaw tape toward the left and right edges -->
		<linearGradient id="{uid}-fade-x-gradient" x1="0" y1="0" x2="1" y2="0">
			<stop offset="0" stop-color="white" stop-opacity="0" />
			<stop offset="0.2" stop-color="white" stop-opacity="1" />
			<stop offset="0.8" stop-color="white" stop-opacity="1" />
			<stop offset="1" stop-color="white" stop-opacity="0" />
		</linearGradient>
		<mask
			id="{uid}-fade-x"
			maskUnits="userSpaceOnUse"
			x={-halfWidth}
			y={-halfHeight}
			{width}
			{height}
		>
			<rect x={-halfWidth} y={-halfHeight} {width} {height} fill="url(#{uid}-fade-x-gradient)" />
		</mask>
	</defs>

	<!-- Crosshair -->
	<line
		x1={-CROSSHAIR_SIZE / 2}
		y1={0}
		x2={CROSSHAIR_SIZE / 2}
		y2={0}
		stroke-width={LINE_THICKNESS}
	/>
	<line
		x1={0}
		y1={-CROSSHAIR_SIZE / 2}
		x2={0}
		y2={CROSSHAIR_SIZE / 2}
		stroke-width={LINE_THICKNESS}
	/>

	<!-- Yaw indicator -->
	<g mask="url(#{uid}-fade-x)" transform="translate(0, {-halfHeight + YAW_SIZE})">
		<polygon
			transform="translate(0, {-YAW_SIZE})"
			points="0,{YAW_SIZE / 2} {YAW_SIZE / 2},{-YAW_SIZE / 2} {-YAW_SIZE / 2},{-YAW_SIZE / 2}"
		/>
		<g transform="translate({-yaw * RAD_TO_DEG * pixelPerDegVertical}, {YAW_SIZE})">
			{#each yawRange as point, i}
				<line
					style:transition="transform 0.1s linear"
					x1={point * pixelPerDegVertical}
					x2={point * pixelPerDegVertical}
					y1={0}
					y2={point % 90 === 0 ? YAW_SIZE : YAW_SIZE / 2}
					stroke-width={LINE_THICKNESS}
				/>
				<text
					x={point * pixelPerDegVertical}
					y={-YAW_SIZE / 2 + (i % 2 === 0 ? 6 : -6)}
					font-family="monospace"
					text-anchor="middle"
					font-size="{pixelPerDegVertical * 6}px"
				>
					{point}
				</text>
			{/each}
			<!-- Zero degree indicator -->
			<line
				x1={0}
				x2={YAW_SIZE / 2}
				y1={YAW_SIZE}
				y2={YAW_SIZE * 1.5}
				stroke-width={LINE_THICKNESS}
			/>
			<line
				x1={0}
				x2={-YAW_SIZE / 2}
				y1={YAW_SIZE}
				y2={YAW_SIZE * 1.5}
				stroke-width={LINE_THICKNESS}
			/>
		</g>
	</g>

	<!-- Roll transform -->
	<g mask="url(#{uid}-fade-y)" transform="rotate({roll * RAD_TO_DEG})">
		<line x1={-halfWidth / 2} y1={0} x2={-CROSSHAIR_SIZE} y2={0} stroke-width={LINE_THICKNESS} />
		<line x1={CROSSHAIR_SIZE} y1={0} x2={halfWidth / 2} y2={0} stroke-width={LINE_THICKNESS} />

		<!-- Pitch -->
		<g transform="translate(0, {pitch * RAD_TO_DEG * pixelPerDegHorizontal})">
			{#each pitchLevels as level}
				<line
					x1={-halfWidth / 4}
					y1={level * pixelPerDegHorizontal}
					x2={-CROSSHAIR_SIZE * 2}
					y2={level * pixelPerDegHorizontal}
					stroke-width={LINE_THICKNESS}
				/>
				<line
					x1={CROSSHAIR_SIZE * 2}
					y1={level * pixelPerDegHorizontal}
					x2={halfWidth / 4}
					y2={level * pixelPerDegHorizontal}
					stroke-width={LINE_THICKNESS}
				/>
				<!-- Adjust text size based on level -->
				<text
					x={halfWidth / 4 - 5}
					y={level * pixelPerDegHorizontal - 5}
					font-family="monospace"
					text-anchor="end"
					font-size="{(pixelPerDegHorizontal / 2) * 6}px"
				>
					{-level}
				</text>
			{/each}
			<!-- Zero degree indicator -->
			<line
				x1={-halfWidth / 4}
				y1={0}
				x2={-halfWidth / 4 - CROSSHAIR_SIZE / 2}
				y2={CROSSHAIR_SIZE / 2}
				stroke-width={LINE_THICKNESS}
			/>
			<line
				x1={-halfWidth / 4 - CROSSHAIR_SIZE / 2}
				y1={-CROSSHAIR_SIZE / 2}
				x2={-halfWidth / 4}
				y2={0}
				stroke-width={LINE_THICKNESS}
			/>
			<line
				x1={halfWidth / 4}
				y1={0}
				x2={halfWidth / 4 + CROSSHAIR_SIZE / 2}
				y2={CROSSHAIR_SIZE / 2}
				stroke-width={LINE_THICKNESS}
			/>
			<line
				x1={halfWidth / 4 + CROSSHAIR_SIZE / 2}
				y1={-CROSSHAIR_SIZE / 2}
				x2={halfWidth / 4}
				y2={0}
				stroke-width={LINE_THICKNESS}
			/>
		</g>
	</g>
</svg>

<style>
	line {
		stroke: currentColor;
	}
	text,
	polygon {
		fill: currentColor;
	}
</style>
