<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'WebRTC Canvas';
	export const description =
		"For viewing multiple video streams that are being handled by the video servers run with 'yarn camera'";
	export const group = 'Gstreamer';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			config: {
				locked: {
					type: 'switch',
					value: 'false',
					description:
						'If true, the layout of the panels in this widget cannot be changed by the user.'
				},
				layout: {
					type: 'text',
					value: '',
					disabled: false,
					description:
						'The layout of the panels in this widget. This is read-only and is set by the user in the panel grid.'
				},
				streams: {
					type: 'text',
					value: '{}',
					disabled: false,
					description:
						'The configuration of the streams in this widget. This is read-only and is set by the user in the panel grid.'
				}
			}
		}
	});

	// Shared types
	export type VideoTransformType =
		| 'none'
		| 'clockwise'
		| 'counterclockwise'
		| 'rotate-180'
		| 'horizontal-flip'
		| 'vertical-flip'
		| 'upper-left-diagonal'
		| 'upper-right-diagonal'
		| 'automatic';

	export interface DeviceType {
		dev: string;
		hostname: string;
		name?: string;
		isVirtual?: boolean;
	}

	interface CameraEventType {
		type: 'camera';
		action:
			| 'group-description'
			| 'kill-stream'
			| 'request-groups'
			| 'request-stream'
			| 'group-terminated'
			| 'device-connect'
			| 'device-disconnect';
		target?: DeviceType;
		devices?: Array<DeviceType>;
		data: {
			resolution?: { width: number; height: number };
			transform?: VideoTransformType;
			forceRestart?: boolean;
			redirect?: DeviceType;
		};
	}

	export interface StreamConfigType {
		name: string;
		device?: DeviceType;
		resolution: string;
		transform: VideoTransformType;
		redirect?: DeviceType;
	}
</script>

<script lang="ts">
	import { movePanel, type Edge, type LayoutNodeType } from '$lib/components/panelGrid/layout';
	import LayoutNode from '$lib/components/panelGrid/layoutNode.svelte';
	import Button from '$lib/components/ui/button/button.svelte';
	import { Cross2, DotsHorizontal, Plus, Reload, Stop, Trash } from 'svelte-radix';
	import { onMount } from 'svelte';
	import * as Select from '$lib/components/ui/select/index';
	import { io, type Socket } from 'socket.io-client';
	import {
		closePeerConnection,
		connectToSignallingServer,
		getPeerConnection,
		getPeerIdString,
		initPeerConnection,
		peerConnections,
		ws
	} from './webrtc/signalHandler.svelte';
	import Label from '$lib/components/ui/label/label.svelte';

	const devicesEqual = (a: DeviceType | undefined, b: DeviceType | undefined): boolean => {
		if (!a || !b) return false;
		return a.dev === b.dev && a.hostname === b.hostname;
	};

	//#region ------------------< panel grid layout >--------------------

	const resolutions: Record<string, { width: number; height: number }> = {
		'320x240': { width: 320, height: 240 },
		'640x480': { width: 640, height: 480 },
		'1280x720': { width: 1280, height: 720 },
		'1920x1080': { width: 1920, height: 1080 }
	};
	const transforms: Array<VideoTransformType> = [
		'none',
		'clockwise',
		'counterclockwise',
		'rotate-180',
		'horizontal-flip',
		'vertical-flip',
		'upper-left-diagonal',
		'upper-right-diagonal',
		'automatic'
	];

	// type alias so its clear what the keys are
	type PanelIdType = string;
	let streamSettings = $state<Record<PanelIdType, StreamConfigType>>({});
	let streamSettingsOpen = $state<Record<PanelIdType, boolean>>({});
	let locked = $derived<boolean>(settings.groups.config.locked.value === 'true');
	let root = $state<LayoutNodeType>({
		type: 'group',
		id: 'root',
		direction: 'horizontal',
		size: 100,
		children: []
	});

	// must save settings as strings using $effects
	// timeout avoids infinite recursion from assigning to settings
	// TODO: move over to widget metadata field
	$effect(() => {
		const stringified = JSON.stringify(root);
		setTimeout(async () => {
			settings.groups.config.layout.value = stringified;
		}, 10);
	});
	$effect(() => {
		const stringified = JSON.stringify(streamSettings);
		setTimeout(async () => {
			settings.groups.config.streams.value = stringified;
		}, 10);
	});

	const onmove = (src: string, target: string, edge: Edge) => {
		root = movePanel(root, src, target, edge);
	};

	const addPanel = () => {
		const newPanel: LayoutNodeType = {
			type: 'panel',
			id: crypto.randomUUID(),
			title: '', // unused
			size: 100
		};
		streamSettings[newPanel.id] = {
			name: 'New Camera',
			resolution: '320x240',
			transform: 'none'
		};
		if (root.type === 'group') {
			root.children.push(newPanel);
		} else {
			root = {
				type: 'group',
				id: 'root',
				direction: 'horizontal',
				size: 100,
				children: [root, newPanel]
			};
		}
	};

	const deletePanel = (panelId: PanelIdType) => {
		// recursively call this function for each node in the tree returning a filtered tree
		const deleteNode = (node: LayoutNodeType): LayoutNodeType | null => {
			if (node.type === 'panel' && node.id === panelId) {
				return null;
			} else if (node.type === 'group') {
				const newChildren = node.children
					.map(deleteNode)
					.filter((child): child is LayoutNodeType => child !== null);
				if (newChildren.length === 0) {
					return null;
				} else if (newChildren.length === 1) {
					return newChildren[0];
				} else {
					return { ...node, children: newChildren };
				}
			}
			return node;
		};
		const newRoot = deleteNode(root);
		if (newRoot) {
			root = newRoot;
		} else {
			root = {
				type: 'group',
				id: 'root',
				direction: 'horizontal',
				size: 100,
				children: []
			};
		}

		// close stream for this panel if it exists
		endStream(panelId);
		delete streamSettings[panelId];
	};

	const togglePanelSettings = (panelId: PanelIdType) => {
		// explicitly set to true if undefined to avoid
		// needing to initialise the map with false
		if (!streamSettingsOpen[panelId]) {
			streamSettingsOpen[panelId] = true;
		} else {
			streamSettingsOpen[panelId] = !streamSettingsOpen[panelId];
		}
	};
	//#endregion

	//#region ------------------< camera server socket >--------------------
	const socket: Socket = io();

	let devices = $state<Array<DeviceType>>([]);
	// filtered lists for dropdowns
	let usbDevices = $derived<Array<DeviceType>>(devices.filter((d) => !d.isVirtual));
	let availableUsbDevices = $derived<Array<DeviceType>>(
		usbDevices.filter((d) => !Object.values(streamSettings).some((s) => devicesEqual(s.device, d)))
	);
	let virtualDevices = $derived<Array<DeviceType>>(devices.filter((d) => d.isVirtual));
	let availableVirtualDevices = $derived<Array<DeviceType>>(
		virtualDevices.filter(
			(d) => !Object.values(streamSettings).some((s) => devicesEqual(s.redirect, d))
		)
	);

	const requestStream = (config: StreamConfigType, forceRestart: boolean = false) => {
		console.log('Requesting stream for', config.device, 'with config', config);
		if (config.device) {
			socket.send({
				type: 'camera',
				action: 'request-stream',
				target: { dev: config.device.dev, hostname: config.device.hostname },
				data: {
					resolution: resolutions[config.resolution],
					transform: config.transform,
					redirect: config.redirect,
					forceRestart
				}
			} as CameraEventType);
			console.log('sending', {
				type: 'camera',
				action: 'request-stream',
				target: { dev: config.device.dev, hostname: config.device.hostname },
				data: {
					resolution: resolutions[config.resolution],
					transform: config.transform,
					redirect: config.redirect,
					forceRestart
				}
			});

			// ensure peer connection exists
			initPeerConnection(config.device);
		}
	};

	const startConfiguredStreams = () => {
		Object.values(streamSettings).forEach((stream) => {
			if (stream.device) {
				requestStream(stream);
			}
		});
	};

	socket.on('camera_event', (event: CameraEventType) => {
		switch (event.action) {
			case 'group-description': {
				if (event.devices) {
					// add missing devices to devices list
					event.devices.forEach((device) => {
						if (!devices.some((d) => devicesEqual(d, device))) {
							devices.push(device);
						}
					});
					startConfiguredStreams();
				}
				break;
			}
			case 'device-connect': {
				if (event.target) {
					// add device to devices list
					if (!devices.some((d) => devicesEqual(d, event.target))) {
						devices.push(event.target);
						startConfiguredStreams();
					}
				}
				break;
			}
			case 'device-disconnect': {
				if (event.target) {
					// remove device from devices list
					devices = devices.filter((d) => !devicesEqual(d, event.target));
					// remove peer connection
					closePeerConnection(event.target);
				}
				break;
			}
			case 'group-terminated': {
				// remove all devices in the group
				event.devices?.forEach((device) => {
					devices = devices.filter((d) => !devicesEqual(d, device));
					// remove peer connection
					closePeerConnection(device);
				});
				break;
			}

			// ignore self sent events
			case 'request-stream':
			case 'request-groups':
			case 'kill-stream':
				break;

			default:
				console.warn('Received camera event', event);
		}
	});

	// Ensure that the peer connections list is synconized with the stream settings
	$effect(() => {
		const streamConfigs = Object.values(streamSettings).filter((s) => s.device);
		setTimeout(async () => {
			// create peer connections if they dont exist
			streamConfigs.forEach((stream) => {
				if (!stream.device) return;
				initPeerConnection(stream.device);
			});

			// Remove any peer connections that are not in the stream settings
			Object.keys(peerConnections).forEach((peerId) => {
				if (!streamConfigs.some((s) => s.device && getPeerIdString(s.device) === peerId)) {
					peerConnections[peerId].connection?.close();
					peerConnections[peerId].track = null;
					delete peerConnections[peerId];
				}
			});
		}, 1);
	});

	const onVideoAction = (
		panelId: string,
		action: keyof CameraEventType['data'] | 'device',
		value: string | boolean | VideoTransformType
	) => {
		const config = streamSettings[panelId];
		if (!config) return;

		switch (action) {
			case 'device':
				// device and redirect are stored as JSON strings doe to shadcn-svelte limitations
				// select component values must be a string
				config.device = JSON.parse(value as string) ?? undefined;
				break;
			case 'resolution':
				config.resolution = value as string;
				break;
			case 'transform':
				config.transform = value as VideoTransformType;
				break;
			case 'redirect':
				config.redirect = JSON.parse(value as string) ?? undefined;
				break;
			default:
				console.warn('Unknown video action:', action);
		}

		requestStream(config, action === 'forceRestart' && value === true);
	};

	const endStream = (panelId: string) => {
		const config = streamSettings[panelId];
		if (config?.device) {
			// tell the server to stop the stream
			socket.send({
				type: 'camera',
				action: 'kill-stream',
				target: config.device,
				data: {}
			} as CameraEventType);

			// close the peer connection
			closePeerConnection(config.device);

			// reset panel config
			config.device = undefined;
			config.redirect = undefined;
			config.resolution = '320x240';
			config.transform = 'none';
		}
	};

	//#endregion

	onMount(() => {
		// load saved config
		if (settings.groups.config.layout.value) {
			try {
				const parsed = JSON.parse(settings.groups.config.layout.value);
				if (parsed) {
					root = parsed;
				}
			} catch (e) {
				console.error('Failed to parse layout from settings', e);
			}
		}
		if (settings.groups.config.streams.value) {
			try {
				const parsed = JSON.parse(settings.groups.config.streams.value);
				if (parsed) {
					streamSettings = parsed;
				}
			} catch (e) {
				console.error('Failed to parse streams from settings', e);
			}
		}

		// request initial group description from camera server
		socket.send({
			type: 'camera',
			action: 'request-groups',
			data: {}
		} as CameraEventType);

		// signalling server is hosted on by the web server process
		connectToSignallingServer(window.location.hostname);
		return () => {
			// Web server socket
			socket.close();
			// signalling server socket
			ws?.close();
			// Close all peer connections
			Object.values(streamSettings).forEach((stream) => {
				if (stream.device) {
					closePeerConnection(stream.device);
				}
			});
		};
	});

	$inspect(peerConnections);
</script>

{#snippet widget(id: string)}
	{@const peer = getPeerConnection(streamSettings[id]?.device)}
	<div class="relative h-full w-full">
		<div class="absolute top-2 right-2 z-20 flex flex-col gap-2">
			<Button variant="outline" size="icon" onclick={() => togglePanelSettings(id)}>
				{#if streamSettingsOpen[id]}
					<Cross2 />
				{:else}
					<DotsHorizontal />
				{/if}
			</Button>
			{#if streamSettingsOpen[id]}
				<Button
					disabled={streamSettings[id]?.device === undefined}
					variant="outline"
					size="icon"
					onclick={() => onVideoAction(id, 'forceRestart', true)}
				>
					<Reload />
				</Button>
				<Button
					disabled={streamSettings[id]?.device === undefined}
					variant="destructive"
					size="icon"
					onclick={() => endStream(id)}
				>
					<Stop />
				</Button>
				<Button disabled={locked} variant="destructive" size="icon" onclick={() => deletePanel(id)}>
					<Trash />
				</Button>
			{/if}
		</div>
		{#if streamSettingsOpen[id]}
			<div class="bg-background/85 absolute top-0 left-0 z-10 h-full w-full">
				<div class="flex h-full w-full flex-col items-center justify-center gap-2 overflow-auto">
					<Label for="device">Device:</Label>
					<Select.Root type="single" onValueChange={(value) => onVideoAction(id, 'device', value)}>
						<Select.Trigger>
							<p>
								{usbDevices.find((d) => devicesEqual(d, streamSettings[id].device))?.name ||
									streamSettings[id].device ||
									'Select device'}
							</p>
						</Select.Trigger>
						<Select.Content>
							<Select.Item value="null">None</Select.Item>
							{#each usbDevices as device}
								<Select.Item
									disabled={!availableUsbDevices.some((d) => devicesEqual(d, device))}
									value={JSON.stringify(device)}
								>
									{device.name || device.dev}
								</Select.Item>
							{:else}
								<p class="mx-2 my-0.5 cursor-default">No available devices</p>
							{/each}
						</Select.Content>
					</Select.Root>
					<Label for="resolution">Resolution:</Label>
					<Select.Root
						type="single"
						onValueChange={(value) => onVideoAction(id, 'resolution', value)}
					>
						<Select.Trigger>
							<p>{streamSettings[id].resolution || 'Select resolution'}</p>
						</Select.Trigger>
						<Select.Content>
							{#each Object.keys(resolutions) as res}
								<Select.Item value={res}>{res}</Select.Item>
							{/each}
						</Select.Content>
					</Select.Root>
					<Label for="transform">Transform:</Label>
					<Select.Root
						type="single"
						onValueChange={(value) => onVideoAction(id, 'transform', value)}
					>
						<Select.Trigger>
							<p>{streamSettings[id].transform || 'Select transform'}</p>
						</Select.Trigger>
						<Select.Content>
							{#each transforms as transform}
								<Select.Item value={transform}>{transform}</Select.Item>
							{/each}
						</Select.Content>
					</Select.Root>
					<Label for="redirect">Redirect:</Label>
					<Select.Root
						type="single"
						onValueChange={(value) => onVideoAction(id, 'redirect', value)}
					>
						<Select.Trigger>
							<p>
								{virtualDevices.find((d) => devicesEqual(d, streamSettings[id].redirect))?.dev ||
									streamSettings[id].redirect ||
									'Select redirect'}
							</p>
						</Select.Trigger>
						<Select.Content>
							<Select.Item value="null">None</Select.Item>
							{#each virtualDevices as device}
								<Select.Item
									disabled={!availableVirtualDevices.some((d) => devicesEqual(d, device))}
									value={JSON.stringify(device)}
								>
									<!-- use dev since name is non descriptive -->
									{device.dev}
								</Select.Item>
							{:else}
								<p class="mx-2 my-0.5 cursor-default">No available virtual devices</p>
							{/each}
						</Select.Content>
					</Select.Root>
				</div>
			</div>
		{/if}
		{#if peer}
			<div class="relative flex h-full w-full justify-center">
				<!-- svelte-ignore a11y_media_has_caption -->
				{#if peer.track}
					<video
						class="max-h-full max-w-full object-contain"
						srcObject={peer.track}
						playsinline
						autoplay
						muted
					></video>
				{:else if !peer.online}
					<p class="m-auto w-[60%] text-center">
						Waiting for {peer.name} to come online...
					</p>
				{:else}
					<p class="text-center">Connecting to {peer.name}...</p>
				{/if}
				<p class="bg-card/60 absolute top-1 left-1 rounded-lg px-2 py-1">
					{peer.name}
				</p>
			</div>
		{:else}
			<div class="flex h-full w-full items-center justify-center">
				<p class="text-center">No stream available for this panel.</p>
			</div>
		{/if}
	</div>
{/snippet}

<div class="relative -m-2 h-[calc(100%+1rem)] w-[calc(100%+1rem)]">
	{#if root.type === 'panel' || (root.type === 'group' && root.children.length > 0)}
		<LayoutNode node={root} {widget} {onmove} {locked} />
	{:else}
		<p class="text-muted-foreground text-center text-sm">Press + to add a panel</p>
	{/if}
	<Button class="absolute bottom-2 left-2" size="icon" onclick={addPanel} disabled={locked}>
		<Plus />
	</Button>
</div>
