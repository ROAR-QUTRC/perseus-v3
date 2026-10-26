<script lang="ts">
	let {
		value,
		target,
		secondaryValue,
		max,
		label,
		secondaryLabel
	}: {
		value: number;
		target: number;
		secondaryValue: number;
		max: number;
		label: string;
		secondaryLabel: string;
	} = $props();

	// Signed arc end as a percentage of the circle: +50 is the bottom-right limit going clockwise over the top, -50 going anticlockwise underneath.
	const toArc = (amount: number) => {
		const end = Math.max(-1, Math.min(1, amount / max)) * 50;
		return {
			end,
			length: Math.abs(end),
			// Shifting the dash start back by the arc length draws negative values anticlockwise from the same start point.
			offset: Math.max(0, -end)
		};
	};

	const valueArc = $derived(toArc(value));
	const targetArc = $derived(toArc(target));
</script>

<svg viewBox="0 0 100 100" class="h-full w-full">
	<circle
		cx="50"
		cy="50"
		r="40"
		fill="none"
		pathLength="100"
		transform="rotate(180 50 50)"
		class="stroke-border"
		stroke-width="4"
	/>
	<circle
		cx="50"
		cy="50"
		r="40"
		fill="none"
		pathLength="100"
		transform="rotate(180 50 50)"
		class="stroke-slate-400"
		stroke-width="2"
		stroke-dasharray="{targetArc.length} {100 - targetArc.length}"
		stroke-dashoffset={targetArc.offset}
	/>
	<circle
		cx="50"
		cy="50"
		r="40"
		fill="none"
		pathLength="100"
		transform="rotate(180 50 50)"
		class="stroke-primary"
		stroke-width="4"
		stroke-dasharray="{valueArc.length} {100 - valueArc.length}"
		stroke-dashoffset={valueArc.offset}
	/>
	<circle
		cx="50"
		cy="50"
		r="40"
		fill="none"
		pathLength="100"
		transform="rotate(180 50 50)"
		class="stroke-white"
		stroke-width="8"
		stroke-dasharray="1 99"
		stroke-dashoffset={0.5 - targetArc.end}
	/>

	<text
		x="68"
		y={52 - 8}
		font-family="monospace"
		text-anchor="end"
		class="fill-white text-[16px] font-bold tabular-nums"
	>
		{value.toFixed(2)}
	</text>

	<!-- m/s stacked as a fraction -->
	<g class="fill-slate-400 text-[6px]" text-anchor="middle">
		<text x="74" y={44 - 8}>m</text>
		<line x1="71" x2="77" y1={47 - 8} y2={47 - 8} class="stroke-slate-400" stroke-width="0.8" />
		<text x="74" y={53 - 8}>s</text>
	</g>
	<text
		x="50"
		y="58"
		font-family="monospace"
		text-anchor="middle"
		class="fill-slate-400 text-[8px]"
	>
		{secondaryLabel}: {String(secondaryValue.toFixed(2)).padStart(5, '0')}
	</text>
	<text x="50" y="68" text-anchor="middle" class="fill-slate-400 text-[8px]">{label}</text>
</svg>
