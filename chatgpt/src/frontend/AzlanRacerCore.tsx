import React, { useEffect, useRef, useState, useCallback } from 'react';
import * as THREE from 'three';
import * as CANNON from 'cannon-es';
import { OrbitControls } from 'three-stdlib';

const MAX_SPEED_KMH = 150;
const MAX_SPEED_MS = MAX_SPEED_KMH / 3.6;
const LINE_Z = 10;
const MIN_LAP_SECONDS = 8;
const SOUTH_GATE_Z = -120;

class CarPhysics {
  constructor(world, config) {
    this.world = world;
    this.config = config;
    this.telemetry = { speed: 0, rpm: 0, throttle: 0, brake: 0, steer: 0, gear: 1 };
    const shape = new CANNON.Box(new CANNON.Vec3(0.9, 0.3, 2.1));
    this.chassis = new CANNON.Body({ mass: config.mass });
    this.chassis.addShape(shape);
    this.chassis.position.set(0, 1.0, LINE_Z);
    world.addBody(this.chassis);
    this.vehicle = new CANNON.RaycastVehicle({ chassisBody: this.chassis, indexRightAxis: 0, indexUpAxis: 1, indexForwardAxis: 2 });
    const baseWheel = {
      radius: 0.42,
      directionLocal: new CANNON.Vec3(0, -1, 0),
      suspensionStiffness: config.suspensionStiffness,
      suspensionRestLength: 0.32,
      frictionSlip: config.tireGrip,
      dampingRelaxation: config.suspensionDamping,
      dampingCompression: config.suspensionDamping * 0.8,
      maxSuspensionForce: 100000,
      rollInfluence: 0.02,
      axleLocal: new CANNON.Vec3(-1, 0, 0),
      chassisConnectionPointLocal: new CANNON.Vec3(),
      maxSuspensionTravel: 0.35,
      customSlidingRotationalSpeed: -30,
      useCustomSlidingRotationalSpeed: true,
    };
    const w1 = { ...baseWheel, chassisConnectionPointLocal: new CANNON.Vec3(-0.85, 0, 1.25) };
    const w2 = { ...baseWheel, chassisConnectionPointLocal: new CANNON.Vec3(0.85, 0, 1.25) };
    const w3 = { ...baseWheel, chassisConnectionPointLocal: new CANNON.Vec3(-0.85, 0, -1.25) };
    const w4 = { ...baseWheel, chassisConnectionPointLocal: new CANNON.Vec3(0.85, 0, -1.25) };
    this.vehicle.addWheel(w1);
    this.vehicle.addWheel(w2);
    this.vehicle.addWheel(w3);
    this.vehicle.addWheel(w4);
    this.vehicle.addToWorld(world);
  }
  update(inputs, dt) {
    const speedMS = this.chassis.velocity.length();
    const cappedThrottle = speedMS >= MAX_SPEED_MS ? 0 : inputs.throttle;
    const engineForce = cappedThrottle * this.config.enginePower * 1000;
    this.vehicle.applyEngineForce(-engineForce, 2);
    this.vehicle.applyEngineForce(-engineForce, 3);
    if (speedMS > MAX_SPEED_MS) {
      const v = this.chassis.velocity.clone();
      if (v.length() > 0) {
        v.normalize();
        const dragMag = Math.min(4000, (speedMS - MAX_SPEED_MS) * 8000);
        const drag = v.scale(-dragMag);
        this.chassis.applyForce(drag, this.chassis.position);
      }
    }
    const brakeForce = inputs.brake * this.config.brakeForce * 1000;
    for (let i = 0; i < 4; i++) this.vehicle.setBrake(brakeForce, i);
    const steer = inputs.steer * this.config.maxSteer;
    this.vehicle.setSteeringValue(steer, 0);
    this.vehicle.setSteeringValue(steer, 1);
    const vlen = this.chassis.velocity.length();
    this.telemetry.speed = vlen * 3.6;
    this.telemetry.rpm = Math.min(8000, Math.max(900, this.telemetry.speed * 90));
    this.telemetry.throttle = inputs.throttle;
    this.telemetry.brake = inputs.brake;
    this.telemetry.steer = inputs.steer;
    this.telemetry.gear = engineForce >= 0 ? 1 : -1;
  }
}

function addWall(scene, world, size, pos) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), new THREE.MeshStandardMaterial({ color: 0x666666 }));
  mesh.position.set(pos[0], pos[1], pos[2]);
  mesh.castShadow = true; mesh.receiveShadow = true; scene.add(mesh);
  const body = new CANNON.Body({ mass: 0 });
  const half = new CANNON.Vec3(size[0] / 2, size[1] / 2, size[2] / 2);
  body.addShape(new CANNON.Box(half));
  body.position.set(pos[0], pos[1], pos[2]);
  world.addBody(body);
}

function buildRoad(scene, world) {
  const h = 2;
  const outerHalfX = 18;
  const innerHalfX = 12;
  const outerNorthZ = 14;
  const outerSouthZ = -150;
  const innerNorthZ = 8;
  const innerSouthZ = -144;
  addWall(scene, world, [1, h, Math.abs(outerNorthZ - outerSouthZ)], [outerHalfX, h / 2, (outerNorthZ + outerSouthZ) / 2]);
  addWall(scene, world, [1, h, Math.abs(outerNorthZ - outerSouthZ)], [-outerHalfX, h / 2, (outerNorthZ + outerSouthZ) / 2]);
  addWall(scene, world, [outerHalfX * 2, h, 1], [0, h / 2, outerNorthZ]);
  addWall(scene, world, [outerHalfX * 2, h, 1], [0, h / 2, outerSouthZ]);
  addWall(scene, world, [1, h, Math.abs(innerNorthZ - innerSouthZ)], [innerHalfX, h / 2, (innerNorthZ + innerSouthZ) / 2]);
  addWall(scene, world, [1, h, Math.abs(innerNorthZ - innerSouthZ)], [-innerHalfX, h / 2, (innerNorthZ + innerSouthZ) / 2]);
  addWall(scene, world, [innerHalfX * 2, h, 1], [0, h / 2, innerNorthZ]);
  addWall(scene, world, [innerHalfX * 2, h, 1], [0, h / 2, innerSouthZ]);
}

function buildTrack(scene, world) {
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshStandardMaterial({ color: 0x2a2a2a }));
  ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true; scene.add(ground);
  const groundBody = new CANNON.Body({ mass: 0 });
  groundBody.addShape(new CANNON.Plane());
  groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
  world.addBody(groundBody);
  const stripe = new THREE.Mesh(new THREE.BoxGeometry(10, 0.2, 1), new THREE.MeshStandardMaterial({ color: 0xffcc00 }));
  stripe.position.set(0, 0.11, LINE_Z);
  stripe.rotation.x = -Math.PI / 2;
  scene.add(stripe);
  buildRoad(scene, world);
}

export default function AzlanRacerCore() {
  const containerRef = useRef(null);
  const [telemetry, setTelemetry] = useState(null);
  const [paused, setPaused] = useState(false);
  const [camMode, setCamMode] = useState('chase');
  const [countdown, setCountdown] = useState(3);
  const [laps, setLaps] = useState(0);
  const [lapTime, setLapTime] = useState(0);
  const [bestLap, setBestLap] = useState(null);
  const carRef = useRef(null);
  const cameraRef = useRef(null);
  const rendererRef = useRef(null);
  const orbitRef = useRef(null);
  const carMeshRef = useRef(null);
  const keys = useRef({ throttle: 0, brake: 0, steer: 0 });
  const lastTimeRef = useRef(0);
  const lapStartRef = useRef(null);
  const prevZRef = useRef(LINE_Z);
  const passedSouthRef = useRef(false);

  useEffect(() => {
    if (!containerRef.current) return;
    const world = new CANNON.World({ gravity: new CANNON.Vec3(0, -9.82, 0) });
    world.solver.iterations = 12;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x88aadd);
    const camera = new THREE.PerspectiveCamera(75, containerRef.current.clientWidth / containerRef.current.clientHeight, 0.1, 1000);
    camera.position.set(0, 4.5, 10);
    cameraRef.current = camera;
    const hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 0.7); scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 0.9); dir.position.set(12, 18, 8); dir.castShadow = true; scene.add(dir);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    renderer.shadowMap.enabled = true;
    containerRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;
    const orbit = new OrbitControls(camera, renderer.domElement);
    orbit.enabled = camMode === 'orbit';
    orbitRef.current = orbit;
    buildTrack(scene, world);
    const carGroup = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.6, 4.2), new THREE.MeshStandardMaterial({ color: 0xe11d48, metalness: 0.6, roughness: 0.3 }));
    body.position.y = 0.5; body.castShadow = true; carGroup.add(body);
    const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.6, 1.6), new THREE.MeshStandardMaterial({ color: 0x111827 }));
    cabin.position.set(0, 0.95, -0.3); carGroup.add(cabin);
    carGroup.position.set(0, 0.2, LINE_Z);
    scene.add(carGroup);
    carMeshRef.current = carGroup;
    const cfg = { mass: 1200, enginePower: 210, maxSteer: 0.55, brakeForce: 280, suspensionStiffness: 28, suspensionDamping: 4.5, tireGrip: 1.25 };
    const car = new CarPhysics(world, cfg);
    carRef.current = car;
    lastTimeRef.current = performance.now();
    let raf = 0;
    const loop = () => {
      const now = performance.now();
      const dt = Math.min(0.033, (now - lastTimeRef.current) / 1000);
      lastTimeRef.current = now;
      if (!paused) {
        const throttleGate = countdown === null ? 1 : 0;
        const gatedInputs = { throttle: keys.current.throttle * throttleGate, brake: keys.current.brake, steer: keys.current.steer };
        carRef.current.update(gatedInputs, dt);
        world.step(1 / 60, dt, 3);
        const p = car.chassis.position; const q = car.chassis.quaternion;
        carMeshRef.current.position.set(p.x, p.y - 0.2, p.z);
        carMeshRef.current.quaternion.set(q.x, q.y, q.z, q.w);
        if (camMode === 'chase') {
          const target = new THREE.Vector3(p.x, p.y + 1.5, p.z);
          const back = new THREE.Vector3(0, 1.8, 6).applyQuaternion(new THREE.Quaternion(q.x, q.y, q.z, q.w));
          camera.position.lerp(target.clone().add(back), 0.15);
          camera.lookAt(target);
        }
        const z = p.z;
        const prevZ = prevZRef.current;
        if (countdown === null) {
          if (z <= SOUTH_GATE_Z) passedSouthRef.current = true;
          const crossed = (prevZ > LINE_Z && z <= LINE_Z) || (prevZ < LINE_Z && z >= LINE_Z);
          if (crossed) {
            const nowSec = now / 1000;
            if (lapStartRef.current === null) {
              lapStartRef.current = nowSec;
              setLaps(0);
              passedSouthRef.current = false;
            } else if (passedSouthRef.current) {
              const lapDuration = nowSec - lapStartRef.current;
              if (lapDuration >= MIN_LAP_SECONDS) {
                setLaps(l => l + 1);
                setBestLap(b => (b === null ? lapDuration : Math.min(b, lapDuration)));
                lapStartRef.current = nowSec;
              }
              passedSouthRef.current = false;
            }
          }
          if (lapStartRef.current !== null) setLapTime((now / 1000) - lapStartRef.current);
        }
        prevZRef.current = z;
        setTelemetry({ ...car.telemetry });
      }
      if (orbitRef.current) orbitRef.current.update();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    const onResize = () => {
      if (!containerRef.current || !rendererRef.current || !cameraRef.current) return;
      const w = containerRef.current.clientWidth, h = containerRef.current.clientHeight;
      rendererRef.current.setSize(w, h);
      cameraRef.current.aspect = w / h; cameraRef.current.updateProjectionMatrix();
    };
    window.addEventListener('resize', onResize);
    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener('resize', onResize);
      if (rendererRef.current && containerRef.current) {
        containerRef.current.removeChild(rendererRef.current.domElement);
        rendererRef.current.dispose();
      }
    };
  }, []);

  useEffect(() => {
    if (!orbitRef.current) return;
    orbitRef.current.enabled = camMode === 'orbit';
  }, [camMode]);

  useEffect(() => {
    if (countdown === null) return;
    if (countdown === 3) {
      const t1 = setTimeout(() => setCountdown(2), 1000);
      const t2 = setTimeout(() => setCountdown(1), 2000);
      const t3 = setTimeout(() => setCountdown('GO'), 3000);
      const t4 = setTimeout(() => setCountdown(null), 3600);
      return () => { clearTimeout(t1); clearTimeout(t2); clearTimeout(t3); clearTimeout(t4); };
    }
  }, [countdown]);

  const onKeyDown = useCallback((e) => {
    const k = e.key.toLowerCase();
    if (k === 'w' || k === 'arrowup') keys.current.throttle = 1;
    if (k === 's' || k === 'arrowdown') keys.current.brake = 1;
    if (k === 'a' || k === 'arrowleft') keys.current.steer = 1;
    if (k === 'd' || k === 'arrowright') keys.current.steer = -1;
    if (k === 'p') setPaused(p => !p);
    if (k === 'c') setCamMode(m => (m === 'chase' ? 'orbit' : 'chase'));
    if (k === 'r') resetCar();
  }, []);

  const onKeyUp = useCallback((e) => {
    const k = e.key.toLowerCase();
    if (k === 'w' || k === 'arrowup') keys.current.throttle = 0;
    if (k === 's' || k === 'arrowdown') keys.current.brake = 0;
    if (k === 'a' || k === 'arrowleft') if (keys.current.steer === 1) keys.current.steer = 0;
    if (k === 'd' || k === 'arrowright') if (keys.current.steer === -1) keys.current.steer = 0;
  }, []);

  useEffect(() => {
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
    };
  }, [onKeyDown, onKeyUp]);

  function resetCar() {
    const car = carRef.current; if (!car) return;
    car.chassis.velocity.setZero();
    car.chassis.angularVelocity.setZero();
    car.chassis.position.set(0, 1.0, LINE_Z);
    car.chassis.quaternion.set(0, 0, 0, 1);
    prevZRef.current = LINE_Z;
    lapStartRef.current = null;
    passedSouthRef.current = false;
    setLapTime(0);
    setLaps(0);
    setBestLap(null);
    setCountdown(3);
  }

  return (
    <div ref={containerRef} className="w-full h-screen bg-black relative select-none">
      <div className="absolute top-3 left-3 z-20 text-white bg-black/50 rounded p-3 font-mono text-sm space-y-1">
        <div>Speed: {telemetry ? telemetry.speed.toFixed(1) : '0.0'} km/h (cap {MAX_SPEED_KMH})</div>
        <div>RPM: {telemetry ? telemetry.rpm.toFixed(0) : '0'}</div>
        <div>Throttle: {telemetry ? Math.round(telemetry.throttle * 100) : 0}% | Brake: {telemetry ? Math.round(telemetry.brake * 100) : 0}%</div>
        <div>Steer: {telemetry ? telemetry.steer.toFixed(2) : '0.00'}</div>
        <div>Lap: {laps} | Time: {lapTime.toFixed(2)}s {bestLap !== null ? `| Best: ${bestLap.toFixed(2)}s` : ''}</div>
        <div>Mode: {camMode === 'chase' ? 'Chase' : 'Orbit'} | P Pause | R Reset | C Camera</div>
      </div>
      {countdown !== null && (
        <div className="absolute inset-0 z-30 grid place-items-center text-white">
          <div className="text-6xl font-black drop-shadow-lg">{countdown === 'GO' ? 'GO!' : countdown}</div>
        </div>
      )}
      {paused && (
        <div className="absolute inset-0 z-40 grid place-items-center text-white bg-black/40">
          <div className="text-3xl font-bold">Paused</div>
        </div>
      )}
    </div>
  );
}
