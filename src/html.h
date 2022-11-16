const char HTML[] PROGMEM = R"(
<head></head>
<body style = 'background-color: #0074D9; font-size: xx-large;'>
    <script language='JavaScript'>
    function formToJson(form){
        var pass=form.pass.value;
        var ssid=form.ssid.value;
        var jsonFormInfo=JSON.stringify({pass:pass, ssid: ssid});
        window.alert(jsonFormInfo);
    }
    document.errorPoints = [];

    function requestLabelUpdate(labelID) {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById(labelID).innerHTML = this.responseText;
        }
      };
      xhttp.open('GET', '/'.concat(labelID), true);
      xhttp.send();
    };

    function requestErrorPlotUpdate(labelID, canvasID) {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            document.getElementById(labelID).innerHTML = this.responseText;

            document.errorPoints.push(parseFloat(this.responseText));
            const canvas = document.getElementById(canvasID);
            var ctx = canvas.getContext("2d");

            ctx.clearRect(0, 0, 500, 500);

            ctx.beginPath();
            ctx.moveTo(0, 50);
            var i = 0;
            while (i < document.errorPoints.length){
                ctx.lineTo(10*i, 50+10*document.errorPoints[i]);
                i++;
            }
            ctx.stroke();

            if (document.errorPoints.length > 20){
                document.errorPoints.shift();
            }
        }
      };
      xhttp.open('GET', '/'.concat(labelID), true);
      xhttp.send();
    };

    setInterval(function ( ) {
      requestLabelUpdate('position1');
      requestLabelUpdate('position2');
      requestLabelUpdate('position3');
      requestLabelUpdate('position4');
      requestLabelUpdate('position5');
    }, 500 ) ;

    setInterval(function ( ) {
      requestLabelUpdate('target1');
      requestLabelUpdate('target2');
      requestLabelUpdate('target3');
      requestLabelUpdate('target4');
      requestLabelUpdate('target5');
    }, 500 ) ;

    setInterval(function ( ) {
      requestLabelUpdate('proportional');
      requestLabelUpdate('integral');
      requestLabelUpdate('derivative');
    }, 500 ) ;

    setInterval(function ( ) {
        requestErrorPlotUpdate('errorDist1', 'canvas1');
        requestErrorPlotUpdate('errorDist2', 'canvas2');
        requestErrorPlotUpdate('errorDist3', 'canvas3');
        requestErrorPlotUpdate('errorDist4', 'canvas4');
        requestErrorPlotUpdate('errorDist5', 'canvas5');
    }, 500 ) ;
    </script>
    <div style = 'margin: 0; position: absolute; top: 0%; left: 5%; -ms-transform: translate(-0%, -0%); transform: translate(-0%, -0%);'>
        <form method='post' action='/settarget' >
            <label class='label'>Target Motor 1:  </label>
            <input type='text' name='setpoint1'/>
            <input type='submit' value='Set'>
        </form>
        <canvas id="canvas1" width="200" height="100" style="border:1px solid #000000;">
        </canvas>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Target:  </span>
            <span id='target1'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Position:  </span>
            <span id='position1'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Error Dist:  </span>
            <span id='errorDist1'> </span>
        </p>
    </div>
    <div style = 'margin: 0; position: absolute; top: 0%; left: 50%; -ms-transform: translate(-50%, -0%); transform: translate(-50%, -0%);'>
        <form method='post' action='/settarget' >
            <label class='label'>Target Motor 2:  </label>
            <input type='text' name='setpoint2'/>
            <input type='submit' value='Set'>
        </form>
        <canvas id="canvas2" width="200" height="100" style="border:1px solid #000000;">
        </canvas>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Target:  </span>
            <span id='target2'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Position:  </span>
            <span id='position2'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Error Dist:  </span>
            <span id='errorDist2'> </span>
        </p>
    </div>
    <div style = 'margin: 0; position: absolute; top: 0%; left: 65%; -ms-transform: translate(-0%, -0%); transform: translate(-0%, -0%);'>
        <form method='post' action='/settarget' >
            <label class='label'>Target Motor 3:  </label>
            <input type='text' name='setpoint3'/>
            <input type='submit' value='Set'>
        </form>
        <canvas id="canvas3" width="200" height="100" style="border:1px solid #000000;">
        </canvas>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Target:  </span>
            <span id='target3'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Position:  </span>
            <span id='position3'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Error Dist:  </span>
            <span id='errorDist3'> </span>
        </p>
    </div>
    <div style = 'margin: 0; position: absolute; top: 50%; left: 20%; -ms-transform: translate(-50%, -0%); transform: translate(-50%, -0%);'>
        <form method='post' action='/settarget' >
            <label class='label'>Target Motor 4:  </label>
            <input type='text' name='setpoint4'/>
            <input type='submit' value='Set'>
        </form>
        <canvas id="canvas4" width="200" height="100" style="border:1px solid #000000;">
        </canvas>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Target:  </span>
            <span id='target4'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Position:  </span>
            <span id='position4'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Error Dist:  </span>
            <span id='errorDist4'> </span>
        </p>
    </div>
    <div style = 'margin: 0; position: absolute; top: 50%; left: 60%; -ms-transform: translate(-50%, -0%); transform: translate(-50%, -0%);'>
        <form method='post' action='/settarget' >
            <label class='label'>Target Motor 5:  </label>
            <input type='text' name='setpoint5'/>
            <input type='submit' value='Set'>
        </form>
        <canvas id="canvas5" width="200" height="100" style="border:1px solid #000000;">
        </canvas>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Target:  </span>
            <span id='target5'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Position:  </span>
            <span id='position5'> </span>
        </p>
        <p>
            <i class='fas fa-tint' style='color:'00add6;'></i>
            <span class='dht-labels'>Error Dist:  </span>
            <span id='errorDist5'> </span>
        </p>
    </div><div style = 'margin: 0; position: absolute; top: 50%; left: 90%; -ms-transform: translate(-50%, -0%); transform: translate(-50%, -0%);'>
        <form method='post' action='/settarget' >
            <label class='label'>P:  </label>
            <input type='text' name='setproportional'/>
            <input type='submit' value='Set'>
            <span id='proportional'> </span>
        </form>
        <form method='post' action='/settarget' >
            <label class='label'>I:  </label>
            <input type='text' name='setintegral'/>
            <input type='submit' value='Set'>
            <span id='integral'> </span>
        </form>
        <form method='post' action='/settarget' >
            <label class='label'>D:</label>
            <input type='text' name='setderivative'/>
            <input type='submit' value='Set'>
            <span id='derivative'> </span>
        </form>
    </div>
    </div><div style = 'margin: 0; position: absolute; top: 90%; left: 80%; -ms-transform: translate(-50%, -0%); transform: translate(-50%, -0%);'>
    <form method='post' action='/settarget' >
        <label class='label'>Mode:</label>
        <input type='text' name='setcontrolmode'/>
        <input type='submit' value='Set'>
        <span id='mode'> </span>
    </form>
</body>
)";
