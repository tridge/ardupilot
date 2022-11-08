#!/bin/bash

cat <<EOF
<html><head><title>Graphs of Delta-Zee Missions</title></head>
<body>
<H1>Delta-Zee Missions</h1>

The following links give key graphs for the 9 missions. The full log files are available on the google drive.

For a 3D viewer and detailed viewing of all data please use <a
href="https://plot.ardupilot.org">plot.ardupilot.org</a>. The files to
view are the ones called "missionN.bin".

<h1>Mission Graphs</h1>

<ul>
EOF
for f in $*; do
  bname=$(basename $f)
  echo "<h2><a href=\"$bname.html\">$f</a></h2>"
  echo "<table><tr><td><img src=\"$bname-map.png\"</img></td><td><img src=\"$bname-map2.png\"</img></td></tr></table><p>"
done
cat <<EOF
<ul>
</body>
</html>
EOF
