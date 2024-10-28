const char HTML_start[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
  <style>
    .h1 {
      font-family: courier, courier-new, serif;
      font-size: 20pt;
      color: blue;
      border-bottom: 2px solid blue;
    }
    .info {
      font-family: courier, arial, verdana, sans-serif;
      background-color: lightgrey;
      width: auto;
      border: 3px solid black;
      padding: 10px;
      margin: 10px;
    }
</style>
<body>

<div class="card">
  <h4>The ESP32 Update web page without refresh</h4><br>
</div>
<script>

setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getData();
}, 2000); //2000mSeconds update rate

function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ADCValue").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "data", true);
  xhttp.send();
}
</script>
)=====";
