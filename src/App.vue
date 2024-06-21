<template>
  <div id="app">
    <h1>ROS and Vue Communication Demo</h1>
    <div>
      <button :class="{ active: activeButton === 'w' }">W</button>
      <button :class="{ active: activeButton === 'a' }">A</button>
      <button :class="{ active: activeButton === 'x' }">X</button>
      <button :class="{ active: activeButton === 'd' }">D</button>
      <button :class="{ active: activeButton === 's' }">S</button>
      <button :class="{ active: activeButton === 'space' }">Space</button>
    </div>
    <div>
      <p>Robot Position:</p>
      <p>X: {{ position.x.toFixed(4) }}</p>
      <p>Y: {{ position.y.toFixed(4) }}</p>
      <p>Z: {{ position.z.toFixed(4) }}</p>
      <p>Current Speed:</p>
      <p>Linear: {{ speed.linear.toFixed(4) }}</p>
      <p>Angular: {{ speed.angular.toFixed(4) }}</p>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue';
import ROSLIB from 'roslib';

const position = ref({ x: 0, y: 0, z: 0 });
const speed = ref({ linear: 0, angular: 0 });
const ros = ref(null);
const cmdVel = ref(null);
const activeButton = ref(null);

const setSpeed = (linear, angular) => {
  const twist = new ROSLIB.Message({
    linear: {
      x: linear,
      y: 0,
      z: 0,
    },
    angular: {
      x: 0,
      y: 0,
      z: angular,
    },
  });

  cmdVel.value.publish(twist);
};

const handleKeydown = (event) => {
  activeButton.value = event.key.toLowerCase();
  let linear = speed.value.linear;
  let angular = speed.value.angular;

  switch (event.key.toLowerCase()) {
    case 'w':
      linear = Math.min(1.0, linear + 0.1);
      break;
    case 'x':
      linear = Math.max(-1.0, linear - 0.1);
      break;
    case 'a':
      angular = Math.min(1.0, angular + 0.1);
      break;
    case 'd':
      angular = Math.max(-1.0, angular - 0.1);
      break;
    case 's':
    case ' ':
      linear = 0;
      angular = 0;
      break;
  }

  setSpeed(linear, angular);
};

const handleKeyup = (event) => {
  if (activeButton.value === event.key.toLowerCase()) {
    activeButton.value = null;
  }
};

onMounted(() => {
  // Initialize ROS connection
  ros.value = new ROSLIB.Ros({
    url: 'ws://localhost:9090',
  });

  ros.value.on('connection', () => {
    console.log('Connected to websocket server.');
  });

  ros.value.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.value.on('close', () => {
    console.log('Connection to websocket server closed.');
  });

  // Initialize cmd_vel topic
  cmdVel.value = new ROSLIB.Topic({
    ros: ros.value,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist',
  });

  // Subscribe to odom topic
  const odomListener = new ROSLIB.Topic({
    ros: ros.value,
    name: '/odom',
    messageType: 'nav_msgs/Odometry',
  });

  odomListener.subscribe((message) => {
    position.value = message.pose.pose.position;
    speed.value.linear = message.twist.twist.linear.x;
    speed.value.angular = message.twist.twist.angular.z;
  });

  // Add event listeners for keydown and keyup
  window.addEventListener('keydown', handleKeydown);
  window.addEventListener('keyup', handleKeyup);
});

onUnmounted(() => {
  // Remove event listeners when component is unmounted
  window.removeEventListener('keydown', handleKeydown);
  window.removeEventListener('keyup', handleKeyup);
});
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  margin-top: 60px;
}

button {
  margin: 5px;
  padding: 10px 20px;
  font-size: 16px;
  cursor: pointer;
}

button.active {
  background-color: #4CAF50;
  color: white;
}
</style>
