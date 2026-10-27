<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Status';
	export const description = 'Monitor the content and vitals of ROS topics';
	export const group = 'ROS';
	export const isRosDependent = false;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				label: {
					type: 'text'
				},
				topic: {
					type: 'select',
					options: [
						{
							label: 'No topics available',
							value: ''
						}
					]
				},
				path: {
					type: 'text',
					description:
						'Path to the value in the message, e.g. "twist.linear.x". If left blank this defaults to "data"'
				},
				condition: {
					type: 'select',
					description: 'Condition used to evaluate the value to a boolean',
					options: [
						{
							label: 'Boolean',
							value: 'boolean'
						},
						{
							label: 'Equals',
							value: 'equals'
						},
						{
							label: 'Comparison',
							value: 'comparison'
						}
					],
					value: 'boolean'
				},
				invertBoolean: {
					type: 'switch',
					description: 'Invert the boolean value',
					value: 'false',
					disabled: true
				},
				equalsValue: {
					type: 'number',
					description: 'Value to compare against for the "Equals" condition',
					value: '0',
					disabled: true
				},
				equalsTolerance: {
					type: 'number',
					description:
						'Tolerance for the "Equals" condition. The value is considered equal if it is within this tolerance of the target value.',
					value: '0',
					disabled: true
				},
				comparisonType: {
					type: 'select',
					description: 'Comparison type for the "Comparison" condition',
					options: [
						{
							label: 'Greater than',
							value: 'greaterThan'
						},
						{
							label: 'Less than',
							value: 'lessThan'
						},
						{
							label: 'Greater than or equal',
							value: 'greaterThanOrEqual'
						},
						{
							label: 'Less than or equal',
							value: 'lessThanOrEqual'
						}
					],
					value: 'greaterThan',
					disabled: true
				},
				comparisonThreshold: {
					type: 'number',
					description: 'Threshold for the "Comparison" condition',
					value: '0',
					disabled: true
				},
				save: {
					type: 'button'
				}
			},
			meta: {
				json: {
					type: 'text'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import * as ROSLIB from 'roslib';
	import * as Tooltip from '$lib/components/ui/tooltip/index';
	import { onMount, untrack } from 'svelte';

	type ConditionType =
		| {
				type: 'boolean';
				invert: boolean;
		  }
		| {
				type: 'equals';
				tolerance: number;
				value: number;
		  }
		| {
				type: 'comparison';
				comparison: 'greaterThan' | 'lessThan' | 'greaterThanOrEqual' | 'lessThanOrEqual';
				threshold: number;
		  };

	let statusConfigs = $state<
		Array<{
			label: string;
			topicName: string;
			messageType?: string;
			topic: ROSLIB.Topic | null;
			// Path to the value in the message, e.g. 'twist.linear.x'
			// If null this defaults to 'data'
			path?: string | null;
			// No messages received on the topic or no topic type found
			inactiveTopic?: boolean;
			// Wrong path or condition type
			invalidConfig?: boolean;
			lastMessage?: number;
			rawValue?: any;
			currentValue?: boolean;
			condition: ConditionType;
		}>
	>([]);

	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			statusConfigs.forEach((status) => {
				ros.getTopicType(status.topicName, (type) => {
					if (!type) {
						status.inactiveTopic = true;
						status.topic = null;
						return;
					} else {
						status.messageType = type;
						status.topic = new ROSLIB.Topic({
							ros: ros,
							name: status.topicName,
							messageType: status.messageType
						});
						status.inactiveTopic = true;
						status.currentValue = false;

						// Subscribe to the topic to receive messages
						status.topic.subscribe((message) => {
							status.lastMessage = Date.now();
							status.rawValue = message;
							status.inactiveTopic = false;

							// get target value from the message
							const path = status.path || 'data';
							const pathParts = path.split('.');
							let currentValue: any = message;
							for (const part of pathParts) {
								if (currentValue && part in currentValue) {
									currentValue = currentValue[part];
								} else {
									currentValue = undefined;
									status.invalidConfig = true;
									break;
								}
							}

							// test target value against the condition
							if (status.condition.type === 'boolean') {
								if (typeof currentValue === 'boolean') {
									status.currentValue = status.condition.invert ? !currentValue : currentValue;
									status.invalidConfig = false;
								} else {
									status.invalidConfig = true;
								}
							} else if (status.condition.type === 'equals') {
								if (typeof currentValue === 'number') {
									const tolerance = status.condition.tolerance;
									const targetValue = status.condition.value;
									status.currentValue = Math.abs(currentValue - targetValue) <= tolerance;
									status.invalidConfig = false;
								} else {
									status.invalidConfig = true;
								}
							} else if (status.condition.type === 'comparison') {
								if (typeof currentValue === 'number') {
									const threshold = status.condition.threshold;
									switch (status.condition.comparison) {
										case 'greaterThan':
											status.currentValue = currentValue > threshold;
											break;
										case 'lessThan':
											status.currentValue = currentValue < threshold;
											break;
										case 'greaterThanOrEqual':
											status.currentValue = currentValue >= threshold;
											break;
										case 'lessThanOrEqual':
											status.currentValue = currentValue <= threshold;
											break;
										default:
											status.currentValue = false; // Default case
									}
									status.invalidConfig = false;
								} else {
									status.invalidConfig = true;
								}
							}
						});
					}
				});
			});
		} else {
			statusConfigs.forEach((status) => {
				status.currentValue = false;
				status.topic = null;
				status.inactiveTopic = true;
			});
		}
	});

	// save the config using a setTimeout to avoid infinite recursion
	$effect(() => {
		const config = JSON.stringify(
			statusConfigs.map(({ label, topicName, path, condition }) => {
				return {
					label,
					topicName,
					path,
					condition
				};
			})
		);

		setTimeout(() => {
			settings.groups.meta.json.value = config;
		}, 0);
	});

	// enable the correct input fields based on the selected condition type
	$effect(() => {
		const conditionType = settings.groups.general.condition.value;
		untrack(() => {
			if (conditionType === 'boolean') {
				settings.groups.general.invertBoolean.disabled = false;
				settings.groups.general.equalsValue.disabled = true;
				settings.groups.general.equalsTolerance.disabled = true;
				settings.groups.general.comparisonType.disabled = true;
				settings.groups.general.comparisonThreshold.disabled = true;
			} else if (conditionType === 'equals') {
				settings.groups.general.invertBoolean.disabled = true;
				settings.groups.general.equalsValue.disabled = false;
				settings.groups.general.equalsTolerance.disabled = false;
				settings.groups.general.comparisonType.disabled = true;
				settings.groups.general.comparisonThreshold.disabled = true;
			} else if (conditionType === 'comparison') {
				settings.groups.general.invertBoolean.disabled = true;
				settings.groups.general.equalsValue.disabled = true;
				settings.groups.general.equalsTolerance.disabled = true;
				settings.groups.general.comparisonType.disabled = false;
				settings.groups.general.comparisonThreshold.disabled = false;
			}
		});
	});

	let topicIntervalHandle: NodeJS.Timeout | null = null;

	onMount(() => {
		// Load the config from the settings
		if (settings.groups.meta.json.value) {
			try {
				const config = JSON.parse(settings.groups.meta.json.value);
				statusConfigs = config.map((item: any) => {
					return {
						label: item.label,
						topicName: item.topicName,
						path: item.path,
						condition: item.condition,
						topic: null
					};
				});
			} catch (error) {
				console.error('Error parsing config:', error);
			}
		}

		// setup the save action
		settings.groups.general.save.action = () => {
			if (!settings.groups.general.label.value) {
				return 'Label is required';
			}
			if (!settings.groups.general.topic.value) {
				return 'Topic is required';
			}
			if (
				settings.groups.general.condition.value === 'equals' &&
				(settings.groups.general.equalsValue.value === '' ||
					settings.groups.general.equalsTolerance.value === '')
			) {
				return 'Equals value and tolerance are required';
			}
			if (
				settings.groups.general.condition.value === 'comparison' &&
				(settings.groups.general.comparisonType.value === '' ||
					settings.groups.general.comparisonThreshold.value === '')
			) {
				return 'Comparison type and threshold are required';
			}

			const conditionType = settings.groups.general.condition.value as
				'boolean' | 'equals' | 'comparison';
			let condition: ConditionType | null = null;
			if (conditionType == 'boolean') {
				condition = {
					type: conditionType,
					invert: settings.groups.general.invertBoolean.value === 'true'
				};
			} else if (conditionType === 'equals') {
				condition = {
					type: conditionType,
					tolerance: settings.groups.general.equalsTolerance.value
						? parseFloat(settings.groups.general.equalsTolerance.value)
						: 0,
					value: settings.groups.general.equalsValue.value
						? parseFloat(settings.groups.general.equalsValue.value)
						: 0
				};
			} else if (conditionType === 'comparison') {
				condition = {
					type: conditionType,
					comparison: settings.groups.general.comparisonType.value as
						'greaterThan' | 'lessThan' | 'greaterThanOrEqual' | 'lessThanOrEqual',
					threshold: settings.groups.general.comparisonThreshold.value
						? parseFloat(settings.groups.general.comparisonThreshold.value)
						: 0
				};
			}

			if (condition) {
				statusConfigs.push({
					label: settings.groups.general.label.value,
					topicName: settings.groups.general.topic.value,
					path: settings.groups.general.path.value,
					condition,
					topic: null
				});
			}

			return 'Successfully added new status monitor';
		};

		// Update the list of available topics every 500ms
		topicIntervalHandle = setInterval(() => {
			const ros = getRosConnection();
			if (ros) {
				// update the list of available topics for the select input
				ros.getTopics(
					({ topics }: { topics: string[]; types: string[] }) => {
						settings.groups.general.topic.options = topics.map((topic) => ({
							label: topic,
							value: topic
						}));
						if (
							!settings.groups.general.topic.options.some(
								(option) => option.value === settings.groups.general.topic.value
							)
						) {
							settings.groups.general.topic.value =
								settings.groups.general.topic.options[0]?.value || '';
						}
					},
					() => {
						settings.groups.general.topic.options = [
							{
								label: 'No topics available',
								value: ''
							}
						];
					}
				);
			} else {
				settings.groups.general.topic.options = [
					{
						label: 'No topics available',
						value: ''
					}
				];
			}
		}, 500);

		return () => {
			if (topicIntervalHandle) {
				clearInterval(topicIntervalHandle);
			}
		};
	});

	const formatLastMessageTime = (timestamp: number) => {
		const elapsedSeconds = Math.floor(Date.now() - timestamp);
		return `${elapsedSeconds}s ago`;
	};
</script>

<ScrollArea class="h-full w-full" orientation="vertical">
	<Tooltip.Provider delayDuration={0}>
		<div class="flex flex-col">
			{#each statusConfigs as status, i}
				<Tooltip.Root>
					<Tooltip.Trigger>
						<div class="flex w-full items-center gap-2">
							<div
								class="mb-0.5 aspect-square h-2.5 shrink-0 rounded-full {status.inactiveTopic
									? 'bg-gray-400'
									: status.currentValue
										? 'bg-green-500'
										: 'bg-red-500'}"
							></div>
							<p
								class="line-clamp-1 font-mono text-xs text-ellipsis {status.inactiveTopic
									? 'italic'
									: ''} {status.invalidConfig ? 'text-red-500' : ''}"
							>
								{status.label}
							</p>
						</div>
					</Tooltip.Trigger>
					<Tooltip.Content side={i === 0 ? 'bottom' : 'top'}>
						<div class="z-50 flex h-fit flex-col">
							{#if status.invalidConfig}
								<p>Invalid config</p>
							{:else if status.inactiveTopic}
								<p>No messages received yet</p>
							{/if}
							{#if status.lastMessage}
								<p>Last message: {formatLastMessageTime(status.lastMessage)}</p>
							{/if}
						</div>
					</Tooltip.Content>
				</Tooltip.Root>
			{/each}
		</div>
	</Tooltip.Provider>
</ScrollArea>
