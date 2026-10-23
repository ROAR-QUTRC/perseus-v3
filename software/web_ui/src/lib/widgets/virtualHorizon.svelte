<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Virtual Horizon';
	export const description = 'View the orientation of the robot using a fighter jet style HUD';
	export const group = 'ROS';
	export const isRosDependent = false;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import HUD from './virtualHorizon/HUD.svelte';
</script>

<div class="relative h-full w-full">
	<HUD />
</div>
