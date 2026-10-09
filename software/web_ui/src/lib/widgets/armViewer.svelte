<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Arm Viewer';
	export const description =
		'3D view of the post-landing arm (drag to orbit, scroll to zoom), animated from /joint_states.';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import * as THREE from 'three';
	import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
	import URDFLoader, { type URDFRobot } from 'urdf-loader';

	// Static copy of payloads/config/arm_model.urdf served by the Svelte app
	const URDF_PATH = '/arm.urdf';

	let container: HTMLDivElement;
	let robot: URDFRobot | null = null;
	let status = $state('Loading model...');

	onMount(() => {
		const scene = new THREE.Scene();
		const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
		const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
		renderer.setPixelRatio(window.devicePixelRatio);
		container.appendChild(renderer.domElement);

		const controls = new OrbitControls(camera, renderer.domElement);
		controls.enableDamping = true;
		controls.dampingFactor = 0.1;

		// Lighting
		scene.add(new THREE.HemisphereLight(0xffffff, 0x555555, 1.4));
		const keyLight = new THREE.DirectionalLight(0xffffff, 1.6);
		keyLight.position.set(0.8, 1.5, 1.2);
		scene.add(keyLight);

		// Ground grid for depth perception (model is rotated to y-up below)
		const grid = new THREE.GridHelper(0.6, 24, 0x666666, 0x444444);
		(grid.material as THREE.Material).transparent = true;
		(grid.material as THREE.Material).opacity = 0.35;
		scene.add(grid);

		new URDFLoader().load(URDF_PATH, (loaded: URDFRobot) => {
			robot = loaded;
			// URDF is z-up; rotate the model upright in three's y-up world
			loaded.rotation.x = -Math.PI / 2;
			scene.add(loaded);

			// Frame the model
			const box = new THREE.Box3().setFromObject(loaded);
			const center = box.getCenter(new THREE.Vector3());
			const size = box.getSize(new THREE.Vector3());
			const maxDim = Math.max(size.x, size.y, size.z) || 0.5;
			camera.near = maxDim / 100;
			camera.far = maxDim * 20;
			camera.position.set(
				center.x + maxDim * 0.9,
				center.y + maxDim * 0.8,
				center.z + maxDim * 1.1
			);
			camera.updateProjectionMatrix();
			controls.target.copy(center);
			controls.update();

			status = 'Model loaded';
		});

		// Render loop
		let frameHandle = requestAnimationFrame(function animate() {
			frameHandle = requestAnimationFrame(animate);
			controls.update();
			renderer.render(scene, camera);
		});

		// Track widget resizes
		const resize = () => {
			const width = container.clientWidth;
			const height = container.clientHeight;
			if (width === 0 || height === 0) return;
			renderer.setSize(width, height);
			camera.aspect = width / height;
			camera.updateProjectionMatrix();
		};
		const observer = new ResizeObserver(resize);
		observer.observe(container);
		resize();

		return () => {
			cancelAnimationFrame(frameHandle);
			observer.disconnect();
			controls.dispose();
			renderer.dispose();
			if (renderer.domElement.parentElement === container) {
				container.removeChild(renderer.domElement);
			}
		};
	});

	// Animate the model from /joint_states
	$effect(() => {
		const ros = getRosConnection();
		if (!ros) {
			status = 'Waiting for ROS...';
			return;
		}

		const sub = new ROSLIB.Topic({
			ros,
			name: '/joint_states',
			messageType: 'sensor_msgs/JointState'
		});
		sub.subscribe((msg: any) => {
			if (!robot) return;
			const values: Record<string, number> = {};
			msg.name?.forEach((joint: string, i: number) => {
				if (robot?.joints[joint] !== undefined) {
					values[joint] = msg.position?.[i] ?? 0;
				}
			});
			robot.setJointValues(values);
			status = 'Live';
		});

		return () => {
			sub.unsubscribe();
		};
	});
</script>

<div bind:this={container} class="h-full w-full overflow-hidden rounded-md"></div>
<p class="pointer-events-none absolute bottom-1 right-2 text-xs opacity-60">{status}</p>
