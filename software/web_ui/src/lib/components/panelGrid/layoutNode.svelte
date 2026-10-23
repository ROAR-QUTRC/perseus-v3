<script lang="ts">
	import * as Resizable from '$lib/components/ui/resizable/index.js';
	import { dnd } from './dnd.svelte';
	import type { LayoutNodeType, Edge } from './layout';
	import type { Snippet } from 'svelte';
	import LayoutNode from './layoutNode.svelte';

	let {
		node,
		locked,
		onmove,
		widget
	}: {
		node: LayoutNodeType;
		locked: boolean;
		onmove: (src: string, target: string, edge: Edge) => void;
		widget: Snippet<[string, string]>;
	} = $props();

	const EDGE_THRESHOLD = 0.3;

	let hover = $state<Edge | null>(null);

	const edgeAt = (e: DragEvent, el: HTMLElement): Edge => {
		const r = el.getBoundingClientRect();
		const x = (e.clientX - r.left) / r.width;
		const y = (e.clientY - r.top) / r.height;
		if (x < EDGE_THRESHOLD) return 'left';
		if (x > 1 - EDGE_THRESHOLD) return 'right';
		if (y < EDGE_THRESHOLD) return 'top';
		if (y > 1 - EDGE_THRESHOLD) return 'bottom';
		return 'center';
	};

	const zone: Record<Edge, string> = {
		left: 'left-0 top-0 h-full w-1/2',
		right: 'right-0 top-0 h-full w-1/2',
		top: 'left-0 top-0 h-1/2 w-full',
		bottom: 'left-0 bottom-0 h-1/2 w-full',
		center: 'inset-0'
	};
</script>

{#if node.type === 'panel'}
	<div
		class="relative flex h-full flex-col border-b text-sm font-medium {locked
			? ''
			: 'cursor-grab active:cursor-grabbing'} {dnd.dragging === node.id
			? 'border-primary border opacity-65'
			: ''}"
		draggable={!locked}
		role="cell"
		tabindex="0"
		ondragover={(e) => {
			if (!dnd.dragging || dnd.dragging === node.id) return;
			e.preventDefault();
			hover = edgeAt(e, e.currentTarget);
		}}
		ondrop={(e) => {
			e.preventDefault();
			if (dnd.dragging && hover) onmove(dnd.dragging, node.id, hover);
			hover = null;
			dnd.dragging = null;
		}}
		ondragstart={(e) => {
			dnd.dragging = node.id;
			e.dataTransfer!.effectAllowed = 'move';
		}}
		ondragleave={() => (hover = null)}
		ondragend={() => (dnd.dragging = null)}
	>
		<div class="min-h-0 flex-1 overflow-auto">{@render widget(node.id, node.title)}</div>

		{#if hover}
			<div
				class="bg-primary/25 ring-primary pointer-events-none absolute z-10 ring-2 {zone[hover]}"
			></div>
		{/if}
	</div>
{:else}
	<!-- re-key so paneforge re-registers panes when children change -->
	{#key node.children.map((c) => c.id).join()}
		<Resizable.PaneGroup direction={node.direction}>
			{#each node.children as child, i (child.id)}
				{#if i > 0}<Resizable.Handle disabled={locked} withHandle={!locked} />{/if}
				<Resizable.Pane
					defaultSize={child.size}
					minSize={10}
					onResize={(size) => (child.size = size)}
				>
					<LayoutNode node={child} {onmove} {widget} {locked} />
				</Resizable.Pane>
			{/each}
		</Resizable.PaneGroup>
	{/key}
{/if}
