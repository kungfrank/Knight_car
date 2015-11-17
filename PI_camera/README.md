This is what I used for a project of mine.

It is a driver + a reader using publish-subscribe using ZMQ + msgpack. The idea is that "publisher" runs as a process that continuously publishes the data using ZMQ. "Subscriber" is the client (running on the same PI) that reads the data using ZMQ. 

This solution allows to run publisher continuously, but interrupt and restart the subscriber.

Suggestion is to use this as it is, and then have subscriber publish the data to ROS.


Requires:

	picamera
	zmq
	msgpack
	msgpack_numpy