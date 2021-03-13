const express = require('express')
const app = express()
const { spawn } = require('child_process');
var path = require('path');
var myParser = require("body-parser");
const port = 8081
var BUSY = false;
//*****************MONDO DB STUFF****************//
const uri = "mongodb://127.0.0.1:27017/WarehouseMap";
const mongoose = require('mongoose');

mongoose.connect(uri, {useNewUrlParser: true, useUnifiedTopology: true});
const db = mongoose.connection;
db.on('error', console.error.bind(console, 'connection error:'));
db.once('open', function() {
  // we're connected!
  console.log("Connected to Database");
});

const { Schema } = mongoose;

var QR = mongoose.model
  ('QR', new Schema({Item: String, Left: String, Right: String}), 'QRs');

// QR.find( function (err,QRs){
//   console.log(QRs);
// })
//*******************END MONGO DB TEST*************//

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname + '/userWeb.html'));
})

app.use(myParser.urlencoded({extended : true}));
app.get('/pickup', (req, res) => {
    var hostname = "http://";
    hostname = hostname + req.get('host');
    var start ="";
    start = "<!DOCTYPE html><html lang='en'><title>Senior Design T311 & T304 Project</title><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><link rel='stylesheet' href='https://www.w3schools.com/w3css/4/w3.css'><link rel='stylesheet' href='https://fonts.googleapis.com/css?family=Lato'><link rel='stylesheet' href='https://fonts.googleapis.com/css?family=Montserrat'><link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css'><body><!-- Navbar --><div class='w3-top'>  <div class='w3-bar w3-blue w3-card w3-left-align w3-large'>    <a href='" + hostname + "' class='w3-bar-item w3-button w3-hide-small w3-padding-large w3-black'>Back</a>  </div>  <!-- Navbar on small screens -->  <div id='navDemo' class='w3-bar-block w3-white w3-hide w3-hide-large w3-hide-medium w3-large'>    <a href='" + hostname + "' class='w3-bar-item w3-button w3-padding-large w3-black'>Back</a>  </div></div><!-- Header --><header class='w3-container w3-light-grey w3-center' style='padding:128px 16px'>";
    var end = "</header><!-- Footer --><footer class='w3-container w3-padding-16 w3-center w3-blue'>    <div class='w3-xlarge '>    <i class='fa fa-facebook-official w3-hover-opacity'></i>    <i class='fa fa-instagram w3-hover-opacity'></i>    <i class='fa fa-snapchat w3-hover-opacity'></i>    <i class='fa fa-pinterest-p w3-hover-opacity'></i>    <i class='fa fa-twitter w3-hover-opacity'></i>    <i class='fa fa-linkedin w3-hover-opacity'></i> </div></footer><script>// Used to toggle the menu on small screens when clicking on the menu buttonfunction myFunction() {  var x = document.getElementById('navDemo');  if (x.className.indexOf('w3-show') == -1) {    x.className += ' w3-show';  } else {     x.className = x.className.replace(' w3-show', '');  }}</script></body></html>";
    var message = "";
    response = {  
        Name:req.query.newID
    };
    QR.findOne({Item: response.Name}, function (err, QRs){
        if (err)
            message = "<h1 style = 'font-size: 40px;'>Error Something went Wrong!</h1>"
        else
        {
            if (QRs != undefined)
            {
                message = "<h1 style = 'font-size: 40px;'>You're package is being delivered!</h1>"

                // const python = spawn('python', ['../Navigation/Nav.py', 'Box1']);
                // python.stdout.on('data', function (data) {
                //     console.log('Pipe data from python script ...');
                //     dataToSend = data.toString();
                //    });
            
                // python.on('close', (code) => {
                //     console.log(`child process close all stdio with code ${code}`); 
                //     res.send(dataToSend)
                // });
                //Add the response to the queue
                console.log(response);
            }
            else
                message = "<h1 style = 'font-size: 40px;'>Sorry! this package is not in stock.</h1>"
        }
        //Run python command
        //res.send(hostname);
        res.send(start + message + end);
    });
})

app.get('/test', (req, res) =>
{
    req.get('host');
    var dataToSend;
    const python = spawn('python', ['script1.py', "Box1"]);
    python.stdout.on('data', function (data) {
        console.log('Pipe data from python script ...');
        dataToSend = data.toString();
        BUSY = true
       });

    python.on('close', (code) => {
       console.log(`child process close all stdio with code ${code}`); 
       BUSY = false
       return res.send(dataToSend)
    });
    res.send(req.get('host'));
});

app.get('/Stocked', (req, res) => {  
    var hostname = "http://";
    hostname = hostname + req.get('host');
    var newElements = new String();
    var first = "<!DOCTYPE html><html lang='en'><title>Senior Design T311 & T304 Project</title><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><link rel='stylesheet' href='https://www.w3schools.com/w3css/4/w3.css'><link rel='stylesheet' href='https://fonts.googleapis.com/css?family=Lato'><link rel='stylesheet' href='https://fonts.googleapis.com/css?family=Montserrat'><link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css'><style>    #customers {      font-family: Arial, Helvetica, sans-serif;      border-collapse: collapse;      width: 100%;    }        #customers td, #customers th {      border: 1px solid #ddd;      padding: 8px;    }        #customers tr:nth-child(even){background-color: #f2f2f2;}        #customers tr:hover {background-color: #ddd;}        #customers th {      padding-top: 12px;      padding-bottom: 12px;      text-align: left;      background-color: #020307;      color: white;    }    </style><body><!-- Navbar --><div class='w3-top'>  <div class='w3-bar w3-blue w3-card w3-left-align w3-large'>    <a href='" + hostname + "' class='w3-bar-item w3-button w3-hide-small w3-padding-large w3-black'>Back</a>  </div>  <!-- Navbar on small screens -->  <div id='navDemo' class='w3-bar-block w3-white w3-hide w3-hide-large w3-hide-medium w3-large'>    <a href='" + hostname + "' class='w3-bar-item w3-button w3-padding-large w3-black'>Back</a>  </div></div><!-- Header --><header class='w3-container w3-light-grey w3-center' style='padding:128px 16px'><table id='customers'><tr><th>Item</th><th>Stock #</th></tr>";
    var last = "</table></header><!-- Footer --><footer class='w3-container w3-padding-16 w3-center w3-blue'>    <div class='w3-xlarge '>    <i class='fa fa-facebook-official w3-hover-opacity'></i>    <i class='fa fa-instagram w3-hover-opacity'></i>    <i class='fa fa-snapchat w3-hover-opacity'></i>    <i class='fa fa-pinterest-p w3-hover-opacity'></i>    <i class='fa fa-twitter w3-hover-opacity'></i>    <i class='fa fa-linkedin w3-hover-opacity'></i> </div></footer><script>// Used to toggle the menu on small screens when clicking on the menu buttonfunction myFunction() {  var x = document.getElementById('navDemo');  if (x.className.indexOf('w3-show') == -1) {    x.className += ' w3-show';  } else {     x.className = x.className.replace(' w3-show', '');  }}</script></body></html>";
    let items = new Map();
    QR.find( function (err,QRs){
        for (let i = 0; i < QRs.length; i++){
            if (!items.has(QRs[i].Item))
            {
                items.set(QRs[i].Item, 1);
            }
            else
            {
                console.log("Incrementing Value");
                items.set(QRs[i].Item,items.get(QRs[i].Item)+ 1)
            }
        }
        items.forEach( (value, key) => {
            newElements = newElements + "<tr>";
            newElements = newElements + "<td>" + key + "</td>";
            newElements = newElements + "<td>" + value + "</td>";
            newElements = newElements + "</tr>";
        });
        res.send(first + newElements + last);
    })
})

app.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`)
})
