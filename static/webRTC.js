
    let peerConnection = new RTCPeerConnection();

    // Create a data channel
    let dataChannel = peerConnection.createDataChannel("dataChannel");

        // Handle data channel events
        dataChannel.onopen = () => console.log('Data channel opened');
        dataChannel.onclose = () => console.log('Data channel closed');
        dataChannel.onerror = (error) => console.error('Data channel error:', error);

    // Send data to the peer
    function sendDataToPeer(data) {
            if (dataChannel.readyState === 'open') {
        dataChannel.send(JSON.stringify(data));
            } else {
        console.error('Data channel is not open');
            }
        }

    // Call this function to send current location data
    function sendCurrentLocationToPeer(latitude, longitude) {
        sendDataToPeer({ latitude, longitude });
        }

    // Your existing code here...

    // Function to get current location and send it to the peer
    function getGeolocation() {
        // Your geolocation logic here...
        navigator.geolocation.getCurrentPosition(position => {
            const latitude = position.coords.latitude;
            const longitude = position.coords.longitude;
            sendCurrentLocationToPeer(latitude, longitude);
        });
        }
