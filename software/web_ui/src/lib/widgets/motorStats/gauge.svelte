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
	const toArcEnd = (amount: number) => Math.max(-1, Math.min(1, amount / max)) * 50;

	const valueEnd = $derived(toArcEnd(value));
	const targetEnd = $derived(toArcEnd(target));
</script>

<svg viewBox="0 0 100 100" class="h-full w-full">
	<circle class="arc track stroke-border" pathLength="100" />
	<circle class="arc target stroke-slate-400" pathLength="100" style:--end={targetEnd} />
	<circle class="arc value stroke-primary" pathLength="100" style:--end={valueEnd} />
	<circle class="arc target-cap stroke-white" pathLength="100" style:--end={targetEnd} />

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

<style>
	.arc {
		cx: 50;
		cy: 50;
		r: 40;
		fill: none;
		transform-box: fill-box;
		transform-origin: center;
		/* Start at middle-left and run clockwise. */
		transform: rotate(180deg);
	}

	.track {
		stroke-width: 4;
	}

	/* The dash pattern repeats every 100, so shifting its start backwards by the arc length
	   draws negative values anticlockwise from the same start point. */
	.value,
	.target {
		--length: max(var(--end), -1 * var(--end));
		stroke-dasharray: var(--length) calc(100 - var(--length));
		stroke-dashoffset: max(0, -1 * var(--end));
	}

	.value {
		stroke-width: 4;
	}

	.target {
		stroke-width: 2;
	}

	/* A 1%-wide dash centred on the target arc end, so it is always normal to the arc. */
	.target-cap {
		stroke-width: 8;
		stroke-dasharray: 1 99;
		stroke-dashoffset: calc(0.5 - var(--end));
	}
</style>
