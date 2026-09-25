<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'WebRTC Video';
	export const description =
		"For viewing multiple video streams that are being handled by the video servers run with 'yarn camera'";
	export const group = 'Gstreamer';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			setupCamera: {
				name: {
					type: 'text',
					description: 'Name of the camera in the group'
				},
				device: {
					type: 'select',
					description: 'Device that video is streamed from.',
					options: []
				},
				create: {
					type: 'button',
					description: 'Create a new camera stream'
				},
				config: {
					type: 'text',
					disabled: true,
					value: '{}'
				}
			}
		}
	});

	export type videoTransformType =
		| 'none'
		| 'clockwise'
		| 'counterclockwise'
		| 'rotate-180'
		| 'horizontal-flip'
		| 'vertical-flip'
		| 'upper-left-diagonal'
		| 'upper-right-diagonal'
		| 'automatic';

	interface CameraEventType {
		type: 'camera';
		action:
			| 'group-description'
			| 'kill'
			| 'request-groups'
			| 'request-stream'
			| 'group-terminated'
			| 'device-disconnect';
		target?: string;
		devices?: Array<string>;
		data?: {
			resolution?: { width: number; height: number };
			transform?: videoTransformType;
			forceRestart?: boolean;
			file?: string;
		};
	}

	export interface ConfigType {
		name: string;
		resolution: {
			width: number;
			height: number;
		};
		transform: videoTransformType;
		file: string | null;
	}
</script>

<script lang="ts">
	import { onMount, untrack } from 'svelte';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import { io, type Socket } from 'socket.io-client';
	import {
		connectToSignallingServer,
		getPeerId,
		newPeerConfigured,
		peerConnections,
		ws
	} from './webrtc/signalHandler.svelte';
	import VideoWrapper from './webrtc/videoWrapper.svelte';

	let socket: Socket = io();

	// The string id is from /dev/v4l/by-id
	let devicesNames = $state<Array<string>>([]);
	let config = $derived<Record<string, ConfigType>>(
		JSON.parse(settings.groups.setupCamera.config.value!) || {}
	);

	const updateAvailableDevices = (device: string, addingNewDevice: boolean) => {
		if (addingNewDevice && !devicesNames.includes(device)) {
			devicesNames.push(device);
			if (!settings.groups.setupCamera.device.options)
				settings.groups.setupCamera.device.options = [];
			settings.groups.setupCamera.device.options.push({
				value: device,
				label: device
					.replace('usb-', '')
					.replace('-video-index0', '')
					.replaceAll('-', ' ')
					.replaceAll('_', ' ')
			});
		} else if (!addingNewDevice && devicesNames.includes(device)) {
			devicesNames = devicesNames.filter((d) => d !== device);
			if (settings.groups.setupCamera.device.options) {
				settings.groups.setupCamera.device.options =
					settings.groups.setupCamera.device.options.filter((option) => option.value !== device);
			}
		}
	};

	// Ensure the peer connection list contains all the configured devices
	$effect(() => {
		// update the peer connections list with the current config
		const devices = Object.keys(config);
		untrack(() => {
			devices.forEach((device) => {
				if (!peerConnections[device]) {
					peerConnections[device] = {
						sessionId: '',
						name: config[device].name,
						online: false,
						connection: null,
						track: null
					};
				}
			});
			// Remove any peer connections that are not in the config
			Object.keys(peerConnections).forEach((device) => {
				if (!devices.includes(device)) {
					peerConnections[device].connection?.close();
					peerConnections[device].track = null;
					delete peerConnections[device];
				}
			});
		});
	});

	// Handle incoming camera events
	socket.on('camera_event', (event: CameraEventType) => {
		switch (event.action) {
			case 'group-description':
				// update ui options
				(event.devices ?? []).forEach((device) => {
					updateAvailableDevices(device, true);
				});

				// request streams for cameras in config
				console.log('Requesting streams:', config, event);
				Object.keys(config).forEach((device) => {
					if (event.devices?.some((d) => device === d)) {
						socket.send({
							type: 'camera',
							action: 'request-stream',
							target: device,
							data: {
								resolution: config[device].resolution,
								transform: config[device].transform,
								file: config[device].file
							}
						} as CameraEventType);
					}
				});
				break;
			case 'device-disconnect':
				// Remove device from the list
				if (event.target) {
					updateAvailableDevices(event.target, false);
				}
				break;
			case 'group-terminated':
				// remove all peer connections for this group
				event.devices?.forEach((device) => {
					if (peerConnections[device]) {
						peerConnections[device].connection?.close();
						peerConnections[device] = {
							sessionId: '',
							name: peerConnections[device].name,
							online: false,
							connection: null,
							track: null
						};
					}
					// Remove device from the list
					updateAvailableDevices(device, false);
				});
				break;
			case 'kill':
				break;
			// Ignore self sent events
			case 'request-groups':
			case 'request-stream':
				break;
			default:
				console.warn('Unknown camera event action:', event.action);
		}
	});

	onMount(() => {
		// Add button actions
		settings.groups.setupCamera.device.options = [];
		settings.groups.setupCamera.create.action = (): string => {
			const values = settings.groups.setupCamera;
			if (!values.name.value || !values.device.value) {
				return 'Please fill in all fields';
			}

			if (config[values.device.value]) {
				return 'Camera with this name already exists';
			}

			config[values.device.value] = {
				name: values.name.value,
				resolution: { width: 320, height: 240 }, // Default resolution
				transform: 'none', // Default transform
				file: null
			};

			// Update settings config field
			settings.groups.setupCamera.config.value = JSON.stringify(config);

			// Send request to create camera
			socket.send({
				type: 'camera',
				action: 'request-stream',
				target: values.device.value,
				data: {
					resolution: config[values.device.value].resolution,
					transform: config[values.device.value].transform,
					file: config[values.device.value].file
				}
			} as CameraEventType);

			// Reset the form
			values.name.value = '';
			values.device.value = '';

			newPeerConfigured();

			return 'Created new camera';
		};

		// send initial request for camera groups
		socket.send({ type: 'camera', action: 'request-groups', data: {} } as CameraEventType);

		connectToSignallingServer(window.location.hostname);
		return () => {
			// Web server socket
			socket.close();
			// signalling server socket
			ws?.close();
			// Close all peer connections
			Object.keys(peerConnections).forEach((key) => {
				peerConnections[key].connection?.close();
				peerConnections[key].track = null;
				delete peerConnections[key];
			});
		};
	});

	// -------------------------------------
	// Video settings functions
	// -------------------------------------

	const onVideoClose = (device: string) => {
		if (peerConnections[device]) {
			peerConnections[device].connection?.close();
			peerConnections[device].track = null;
			peerConnections[device].online = false;
			delete peerConnections[device];
		}

		// Remove from config
		if (config[device]) {
			delete config[device];
			settings.groups.setupCamera.config.value = JSON.stringify(config);
		}
	};

	const onVideoSettingsChange = (device: string, newConfig: ConfigType) => {
		// Update the config
		config[device] = newConfig;
		settings.groups.setupCamera.config.value = JSON.stringify(config);

		// Send request to update stream
		socket.send({
			type: 'camera',
			action: 'request-stream',
			target: device,
			data: {
				resolution: newConfig.resolution,
				transform: newConfig.transform,
				file: newConfig.file
			}
		} as CameraEventType);
	};

	const onVideoRestart = (device: string) => {
		socket.send({
			type: 'camera',
			action: 'request-stream',
			target: device,
			data: {
				resolution: config[device].resolution,
				transform: config[device].transform,
				file: config[device].file,
				forceRestart: true
			}
		} as CameraEventType);
	};
</script>

<ScrollArea orientation="vertical" class="relative flex h-full w-full">
	<p class="absolute bottom-1 left-1 rounded-[4px] bg-card bg-opacity-60 px-2 py-1">
		Session ID: {getPeerId()}
	</p>
	{#if Object.keys(peerConnections).length === 0}
		<div
			class="absolute left-1/2 top-1/2 flex -translate-x-1/2 -translate-y-1/2 flex-col items-center justify-center"
		>
			<p class="text-center">No video streams available.</p>
			<p class="text-center">Please create a new camera stream in the settings.</p>
		</div>
	{/if}
	<div class="flex flex-row flex-wrap">
		{#each Object.keys(peerConnections) as peer}
			<div class="relative m-2 min-h-[320px] min-w-[480px] overflow-hidden rounded-lg border">
				<VideoWrapper
					device={peer}
					config={config[peer]}
					{onVideoClose}
					{onVideoSettingsChange}
					{onVideoRestart}
				/>
			</div>
		{/each}
	</div>
</ScrollArea>
