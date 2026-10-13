<script module lang="ts">
	export interface CanvasApi {
		drawLine(x1: number, y1: number, x2: number, y2: number, color?: string, width?: number): void;
		drawLineAngle(
			x: number,
			y: number,
			angleDeg: number,
			length: number,
			color?: string,
			width?: number
		): { x: number; y: number };
		drawPoint(x: number, y: number, radius?: number, color?: string): void;
		setOpacity(opacity: number): void;
		eventToWorld(e: MouseEvent): { x: number; y: number };
		fillTriangle(
			x1: number,
			y1: number,
			x2: number,
			y2: number,
			x3: number,
			y3: number,
			color?: string
		): void;
		fillRect(x1: number, y1: number, x2: number, y2: number, color?: string): void;
		clear(): void;
	}
</script>

<script lang="ts">
	// Origin (0,0) at the centre, +x right, +y up. The world is a fixed
	// worldWidth x worldHeight space scaled to fit, so the aspect ratio never changes.
	let {
		worldWidth = 800,
		worldHeight = 600,
		api = $bindable<CanvasApi | null>(null),
		ondraw
	}: {
		worldWidth?: number;
		worldHeight?: number;
		api?: CanvasApi | null;
		ondraw?: (api: CanvasApi) => void;
	} = $props();

	let canvas: HTMLCanvasElement;
	let ctx: CanvasRenderingContext2D;
	let w = $state(0); // CSS size, kept up to date by Svelte's size bindings
	let h = $state(0);

	const scale = $derived(Math.min(w / worldWidth, h / worldHeight));

	function drawLine(x1: number, y1: number, x2: number, y2: number, color = '#fff', width = 2) {
		ctx.beginPath();
		ctx.moveTo(x1, y1);
		ctx.lineTo(x2, y2);
		ctx.strokeStyle = color;
		ctx.lineWidth = width; // world units
		ctx.lineCap = 'square';
		ctx.stroke();
	}

	// Angle in degrees, counter-clockwise from +x. Returns the end point.
	function drawLineAngle(
		x: number,
		y: number,
		angleDeg: number,
		length: number,
		color?: string,
		width?: number
	) {
		const rad = (angleDeg * Math.PI) / 180;
		const end = { x: x + Math.cos(rad) * length, y: y + Math.sin(rad) * length };
		drawLine(x, y, end.x, end.y, color, width);
		return end;
	}

	function eventToWorld(e: MouseEvent) {
		const r = canvas.getBoundingClientRect();
		return {
			x: (e.clientX - r.left - r.width / 2) / scale,
			y: -(e.clientY - r.top - r.height / 2) / scale
		};
	}

	function clear() {
		ctx.save();
		ctx.setTransform(1, 0, 0, 1, 0, 0); // pixel space
		ctx.clearRect(0, 0, canvas.width, canvas.height);
		ctx.restore();
	}

	function drawPoint(x: number, y: number, radius = 5, color = '#fff') {
		ctx.beginPath();
		ctx.arc(x, y, radius, 0, 2 * Math.PI);
		ctx.fillStyle = color;
		ctx.fill();
	}

	function setOpacity(opacity: number) {
		ctx.globalAlpha = opacity;
	}
	function fillTriangle(
		x1: number,
		y1: number,
		x2: number,
		y2: number,
		x3: number,
		y3: number,
		color = '#fff'
	) {
		ctx.beginPath();
		ctx.moveTo(x1, y1);
		ctx.lineTo(x2, y2);
		ctx.lineTo(x3, y3);
		ctx.closePath();
		ctx.fillStyle = color;
		ctx.fill();
	}

	// Two opposite corners, e.g. bottom-left and top-right. Axis-aligned.
	function fillRect(x1: number, y1: number, x2: number, y2: number, color = '#fff') {
		ctx.fillStyle = color;
		ctx.fillRect(Math.min(x1, x2), Math.min(y1, y2), Math.abs(x2 - x1), Math.abs(y2 - y1));
	}

	$effect(() => {
		ctx = canvas.getContext('2d')!;
		api = {
			drawLine,
			drawLineAngle,
			eventToWorld,
			clear,
			drawPoint,
			setOpacity,
			fillTriangle,
			fillRect
		};
		return () => (api = null);
	});

	// Re-runs on resize and whenever any $state read inside ondraw changes.
	$effect(() => {
		if (!api || !w || !h) return;
		const dpr = devicePixelRatio;
		canvas.width = w * dpr; // assigning the size also clears the canvas
		canvas.height = h * dpr;
		ctx.setTransform(scale * dpr, 0, 0, -scale * dpr, canvas.width / 2, canvas.height / 2);
		api.clear();
		api.setOpacity(1.0);
		ondraw?.(api);
	});
</script>

<canvas bind:this={canvas} bind:clientWidth={w} bind:clientHeight={h}></canvas>

<style>
	canvas {
		display: block;
		width: 100%;
		height: 100%;
	}
</style>
