// ws://54.147.26.233:9090

var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        ws_address: 'ws://192.168.137.107:9090',
        logs: [],
        loading: false,
        topic: null,
        message: null,
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
            })
        },
        disconnect: function() {
            this.ros.close()
        },
        setTopic: function() {
            this.topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
        },
        forward: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 1.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        stop: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        backward: function() {
            this.message = new ROSLIB.Message({
                linear: { x: -1.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnLeft: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 1.0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnRight: function() {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -1.0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
    },
    mounted() {
    },
})