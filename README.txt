To start the server:

1.) Start mongoDB Windows
	a.) goto C:\Program Files\MongoDB\Server\4.4\bin and start mongod.exe
	b.) goto C:\Program Files\MongoDB\Server\4.4\bin and start mongo.exe

2.) Start Mongo Db on Pi Ubuntu
	# Ensure mongod config is picked up:
	sudo systemctl daemon-reload

	# Tell systemd to run mongod on reboot:
	sudo systemctl enable mongod

	# Start up mongod!
	sudo systemctl start mongod

	$ sudo systemctl status mongod

	● mongod.service - MongoDB Database Server
	Loaded: loaded (/lib/systemd/system/mongod.service; enabled; vendor preset: enabled)
	Active: active (running) since Tue 2020-08-09 08:09:07 UTC; 4s ago
	Docs: https://docs.mongodb.org/manual
	Main PID: 2366 (mongod)
	CGroup: /system.slice/mongod.service
			└─2366 /usr/bin/mongod --config /etc/mongod.conf

3.) Run "node test.js" in the cmd prompt

4.) Ip Tunneling:

	For IP Tunneling, we can use ngrok

	username: wac18@my.fsu.edu
	password: Team311rocks



	ngrok user manual: https://ngrok.com/docs

	We need to download ngrok to the Pi and then extract the app to the folder
	we plan on using it in
	
	Command for authorization token on terminal:
	ngrok authtoken 1oP8Tpdmc4FtedsHibS9U3MuTj5_43uAPzZrDYSQeyG33ikSe
	
	Create a tunnel to my localhost using: 
	#USE PORT THE SERVER IS HOSTED ON
	ngrok 80 | ngrok http 80

	Command to SSH to the Pi:       //allow us to access the Pi remotely
	ngrok tcp 22

	<u>Getting started</u>
	https://projects.raspberrypi.org/en/projects/raspberry-pi-getting-started
