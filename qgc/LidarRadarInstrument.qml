// Lidar radar for the QGroundControl Fly view (OBSTACLE_DISTANCE from PX4 or ArduPilot).
// Achraf Rahibe - RPLidar obstacle avoidance project.
//
// Install: in QGroundControl's ini file (QGC closed)
//   [FlyView]
//   instrumentQmlFile2=file:///D:/Dev/LIDAR/qgc/LidarRadarInstrument.qml
//
// The radar sits above QGC's attitude/compass instrument, at the same width.
// Drag it to move it, double-click to put it back. Mouse wheel or -/+ to zoom (1.5/3/6/12 m).
// Front is up and matches the flight controller's arrow (the scan is in the body frame).
// Red inside CP_DIST (read from the vehicle, 2 m if missing), yellow up to 2 x CP_DIST, green beyond.
import QtQuick
import QtQuick.Window
import QtCore as QtC

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FactControls

Column {
    id:         root
    spacing:    ScreenTools.defaultFontPixelHeight * 0.3

    // read by QGC's instrument panel; taken from the attitude instrument we embed
    property real extraInset:       attitude.item && attitude.item.extraInset !== undefined ? attitude.item.extraInset : 0
    property real extraValuesWidth: attitude.item && attitude.item.extraValuesWidth !== undefined ? attitude.item.extraValuesWidth : 0

    readonly property string ownerName: "Achraf Rahibe"
    readonly property var    ranges:    [1.5, 3, 6, 12]

    property var  _vehicle:         QGroundControl.multiVehicleManager.activeVehicle
    property var  _oa:              _vehicle ? _vehicle.objectAvoidance : null
    property real _lastUpdateMs:    0
    property real radarSize:        attitude.item && attitude.item.width > 0 ? attitude.item.width
                                                                             : ScreenTools.defaultFontPixelHeight * 14

    QtC.Settings {
        id:         saved
        category:   "LidarRadarWidget"
        property bool moved:        false
        property real offsetX:      0       // from the panel's bottom-right corner
        property real offsetY:      0
        property int  rangeIndex:   2
        property real sizeScale:    1.0     // radar size relative to the attitude instrument
    }

    property bool fullScreen:   false
    readonly property real minScale: 0.7
    readonly property real maxScale: 4.0

    function resize(factor) {
        saved.sizeScale = Math.max(minScale, Math.min(maxScale, saved.sizeScale * factor))
    }

    // radar edge length: full screen fills the Fly view, otherwise the attitude width x sizeScale
    property real boxSize: fullScreen && _host ? Math.min(_host.width, _host.height) * 0.9
                                               : radarSize * saved.sizeScale

    function cpDist() {
        var f = param("CP_DIST")
        if (f && f.rawValue > 0)
            return f.rawValue
        return 2.0
    }

    function vehicleKind() {
        // MAV_TYPE when QGC exposes it, otherwise QGC's vehicle class flags
        var t = -1
        try { if (_vehicle && _vehicle.vehicleType !== undefined) t = _vehicle.vehicleType } catch (e) { }
        if (t === 1 || t === 16) return "plane"
        if (t === 4) return "heli"
        if (t === 13) return "hexa"
        if (t === 14) return "octo"
        if (t === 15) return "tri"
        if (t === 3 || t === 2) return "quad"
        if (t >= 19 && t <= 25) return "vtol"
        if (t === 10) return "rover"
        if (t === 11 || t === 12) return "boat"
        if (_vehicle) {
            if (_vehicle.vtol) return "vtol"
            if (_vehicle.fixedWing) return "plane"
            if (_vehicle.rover) return "rover"
            if (_vehicle.sub) return "boat"
        }
        return "quad"
    }

    // ---- position source (GPS / GPS + optical flow / optical flow), same parameters as PX4's aid_switch
    property int srcMode: -1        // 0 GPS, 1 GPS + flow, 2 flow, -1 unknown / both off
    // vehicle parameters through QGC's FactPanelController (ParameterManager's methods are not
    // callable from QML in QGC 5). The controller binds to the vehicle that is active when it is
    // created, so it is created once a vehicle is connected and again when the vehicle changes.
    Component { id: factControllerComponent; FactPanelController { } }
    property var _factController: null
    property var _factControllerVehicle: null
    function param(name) {
        if (!_vehicle)
            return null
        if (!_factController || _factControllerVehicle !== _vehicle) {
            if (_factController)
                _factController.destroy()
            _factController = factControllerComponent.createObject(root)
            _factControllerVehicle = _vehicle
        }
        try {
            if (_factController && _factController.parameterExists(-1, name))
                return _factController.getParameterFact(-1, name, false)
        } catch (e) { }
        return null
    }
    // ---- optical flow / no-GPS readiness (properties only: getFact() from QML crashed QGC 5)
    property real   flowHeight:     NaN     // m, downward distance sensor
    property bool   posOk:          false   // EKF has a valid relative position and velocity
    property bool   px4Ready:       false   // PX4 preflight checks passed
    property bool   px4ReadyKnown:  false
    property string readyText:      ""
    property color  readyColor:     "#d64545"
    function readFlow() {
        var h = NaN
        try { h = _vehicle.distanceSensors.rotationPitch270.rawValue } catch (e) { }
        flowHeight = (typeof h === "number" && isFinite(h) && h > 0) ? h : NaN
        var p = false, v = false
        try { p = _vehicle.estimatorStatus.goodHorizPosRelEstimate.rawValue === true || _vehicle.estimatorStatus.goodHorizPosRelEstimate.rawValue === 1 } catch (e) { }
        try { v = _vehicle.estimatorStatus.goodHorizVelEstimate.rawValue === true || _vehicle.estimatorStatus.goodHorizVelEstimate.rawValue === 1 } catch (e) { }
        posOk = p && v
        // same source as QGC's toolbar "Ready": on PX4 readyToFlyAvailable is false and the toolbar
        // uses the health and arming check report instead
        try {
            px4ReadyKnown = true
            if (_vehicle.armed)                    px4Ready = true
            else if (_vehicle.readyToFlyAvailable) px4Ready = _vehicle.readyToFly
            else                                   px4Ready = _vehicle.healthAndArmingCheckReport.canArm
        } catch (e) { px4ReadyKnown = false; px4Ready = false }

        var maxH = 6.0
        var m = param("SENS_FLOW_MAXHGT")
        if (m && m.rawValue > 0) maxH = m.rawValue
        if (!_vehicle) { readyText = ""; return }
        if (srcMode === 0)              { readyText = "GPS ONLY: FLOW OFF";                 readyColor = "#8a96a3"; return }
        if (srcMode < 0)                { readyText = "NO POSITION SOURCE";                 readyColor = "#d64545"; return }
        if (isNaN(flowHeight))          { readyText = "NO HEIGHT FROM FLOW SENSOR";         readyColor = "#d64545"; return }
        if (flowHeight > maxH)          { readyText = "TOO HIGH FOR FLOW (> " + maxH.toFixed(0) + " m)"; readyColor = "#d64545"; return }
        if (!posOk)                     { readyText = "NO VALID POSITION YET";              readyColor = "#d64545"; return }
        if (!px4Ready)                  { readyText = "POSITION OK, PX4 NOT READY";         readyColor = "#ff9f1a"; return }
        readyText = srcMode === 2 ? "READY: NO GPS" : "READY: GPS + FLOW"
        readyColor = "#2fb35a"
    }

    function readSource() {
        var g = param("EKF2_GPS_CTRL"), o = param("EKF2_OF_CTRL")
        if (!g || !o) { srcMode = -1; return }
        var gps = g.rawValue !== 0, of = o.rawValue !== 0
        srcMode = gps && !of ? 0 : (gps && of ? 1 : (!gps && of ? 2 : -1))
    }
    function setSource(m) {
        var g = param("EKF2_GPS_CTRL"), o = param("EKF2_OF_CTRL")
        if (!g || !o) return
        var a = param("ASW_GPS_CTRL")
        var gpsOn = a && a.rawValue > 0 ? a.rawValue : 7
        var gWant = m === 2 ? 0 : gpsOn
        var oWant = m === 0 ? 0 : 1
        if (g.rawValue !== gWant) g.rawValue = gWant
        if (o.rawValue !== oWant) o.rawValue = oWant
        srcMode = m
    }

    function zoom(step) {
        saved.rangeIndex = Math.max(0, Math.min(ranges.length - 1, saved.rangeIndex + step))
    }

    // The Fly view's top-level item (the child of the window's content item that contains us).
    // Drawing the radar there keeps it above the instrument panel's mouse area, so it can be
    // dragged, while other pages (Plan, Analyze, Vehicle Configuration) still cover it.
    property Item _host: null
    function findHost() {
        var w = root.Window.window
        if (!w)
            return null
        var p = root
        while (p.parent && p.parent !== w.contentItem)
            p = p.parent
        return p.parent ? p : null
    }

    function place() {
        if (!_host)
            _host = findHost()
        if (drag.active || grip.active || !_host || radarBox.parent !== _host)
            return
        if (fullScreen) {
            radarBox.x = (_host.width - radarBox.width) / 2
            radarBox.y = (_host.height - radarBox.height) / 2
        } else if (saved.moved) {
            var c = root.mapToItem(radarBox.parent, root.width, root.height)
            radarBox.x = c.x + saved.offsetX
            radarBox.y = c.y + saved.offsetY
        } else {
            // docked: bottom-right corner of the radar on the slot's bottom-right corner
            // docked: right edge where the attitude instrument's right edge is; grows to the left
            var p = slot.mapToItem(radarBox.parent, 0, slot.height)
            radarBox.x = p.x + root.radarSize - radarBox.width
            radarBox.y = p.y - radarBox.height
        }
    }

    Connections {
        target:                 root._oa
        ignoreUnknownSignals:   true
        function onObjectAvoidanceChanged() { root._lastUpdateMs = Date.now() }
    }

    Timer {
        interval:       100
        running:        true
        repeat:         true
        onTriggered:    { root.place(); radar.requestPaint(); root.readSource(); root.readFlow() }
    }

    // keeps the space above the attitude instrument while the radar is docked
    Item {
        id:     slot
        width:  root.radarSize
        height: saved.moved ? 0 : root.radarSize * saved.sizeScale
    }

    // corner button size grows with the radar
    property real buttonSize: ScreenTools.defaultFontPixelHeight * 1.1 * Math.max(1, Math.sqrt(boxSize / radarSize))

    // QGC's own attitude and compass instrument
    Loader {
        id:     attitude
        source: "qrc:/qml/QGroundControl/FlightMap/Widgets/VerticalCompassAttitude.qml"
    }

    // The radar is drawn on the Fly view's top-level item (see findHost) so it can be dragged.
    Item {
        id:         radarBox
        parent:     root._host ? root._host : root
        width:      root.boxSize
        height:     root.boxSize
        z:          1000
        visible:    root.visible

        // dark backdrop over the whole Fly view in full screen; click it to leave full screen
        Rectangle {
            x:          -radarBox.x
            y:          -radarBox.y
            width:      root._host ? root._host.width : 0
            height:     root._host ? root._host.height : 0
            z:          -1
            color:      "#e0000000"
            visible:    root.fullScreen
            TapHandler { onTapped: root.fullScreen = false }
        }

        DragHandler {
            id:      drag
            target:  radarBox
            enabled: !root.fullScreen && !gripHover.hovered   // the grip resizes, it must not move
            onActiveChanged: {
                if (!active) {
                    var c = root.mapToItem(radarBox.parent, root.width, root.height)
                    saved.offsetX = radarBox.x - c.x
                    saved.offsetY = radarBox.y - c.y
                    saved.moved = true
                }
            }
        }
        TapHandler {
            onDoubleTapped: { root.fullScreen = false; saved.moved = false; saved.offsetX = 0; saved.offsetY = 0 }
        }
        WheelHandler {                                  // wheel: range
            acceptedModifiers: Qt.NoModifier
            onWheel: (event) => root.zoom(event.angleDelta.y > 0 ? -1 : 1)
        }
        WheelHandler {                                  // Ctrl + wheel: size
            acceptedModifiers: Qt.ControlModifier
            onWheel: (event) => root.resize(event.angleDelta.y > 0 ? 1.1 : 1 / 1.1)
        }

        Canvas {
            id:             radar
            anchors.fill:   parent

            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                // text and lines grow with the radar
                var fs = ScreenTools.defaultFontPixelHeight * Math.max(0.8, width / root.radarSize)
                var W = width, cx = W / 2, cy = W / 2
                var R = W / 2 - fs * 0.55
                var cp = root.cpDist()
                var range = root.ranges[saved.rangeIndex]
                var stale = !root._oa || !root._oa.available || (Date.now() - root._lastUpdateMs) > 1000

                function px(m) { return R * Math.min(m, range) / range }
                function rad(deg) { return (deg - 90) * Math.PI / 180 }       // 0 deg = up, clockwise
                function pt(deg, r) { var a = deg * Math.PI / 180; return [cx + r * Math.sin(a), cy - r * Math.cos(a)] }
                function col(m) { return m < cp ? "#ff453a" : (m < 2 * cp ? "#ffd60a" : "#30d158") }
                function shade(m) { return m < cp ? "rgba(255,69,58,0.22)" : (m < 2 * cp ? "rgba(255,214,10,0.15)" : "rgba(48,209,88,0.12)") }
                function text(t, x, y, color, size, bold, align) {
                    ctx.font = (bold ? "bold " : "") + Math.max(7, Math.round(size)) + "px sans-serif"
                    var w = ctx.measureText(t).width
                    var x0 = align === "left" ? x : (align === "right" ? x - w : x - w / 2)
                    ctx.fillStyle = color
                    ctx.fillText(t, x0, y)
                }
                function pill(t, x, y, color, size) {
                    ctx.font = "bold " + Math.max(7, Math.round(size)) + "px sans-serif"
                    var w = ctx.measureText(t).width
                    ctx.fillStyle = "rgba(10,12,15,0.75)"
                    ctx.fillRect(x - w / 2 - 3, y - size * 0.9, w + 6, size * 1.2)
                    ctx.fillStyle = color
                    ctx.fillText(t, x - w / 2, y)
                }

                // body: dark glass disc with a bezel
                var g = ctx.createRadialGradient(cx, cy, R * 0.1, cx, cy, W / 2)
                g.addColorStop(0, "rgba(28,34,40,0.97)")      // nearly opaque: map must not show through
                g.addColorStop(1, "rgba(12,15,18,0.98)")
                ctx.fillStyle = g
                ctx.beginPath(); ctx.arc(cx, cy, W / 2 - 1, 0, 2 * Math.PI); ctx.fill()
                ctx.lineWidth = 1.5
                ctx.strokeStyle = "rgba(160,176,192,0.55)"
                ctx.beginPath(); ctx.arc(cx, cy, W / 2 - 1, 0, 2 * Math.PI); ctx.stroke()

                // range rings and ticks every 30 degrees
                ctx.lineWidth = 1
                ctx.strokeStyle = "rgba(120,136,150,0.35)"
                for (var q = 1; q <= 4; q++) {
                    ctx.beginPath(); ctx.arc(cx, cy, R * q / 4, 0, 2 * Math.PI); ctx.stroke()
                }
                for (var tdeg = 0; tdeg < 360; tdeg += 30) {
                    var t0 = pt(tdeg, R), t1 = pt(tdeg, R - fs * (tdeg % 90 === 0 ? 0.5 : 0.3))
                    ctx.beginPath(); ctx.moveTo(t0[0], t0[1]); ctx.lineTo(t1[0], t1[1]); ctx.stroke()
                }
                // CP_DIST and 2 x CP_DIST boundaries
                ctx.lineWidth = 1.5
                if (cp <= range) {
                    ctx.strokeStyle = "rgba(255,69,58,0.75)"
                    ctx.beginPath(); ctx.arc(cx, cy, px(cp), 0, 2 * Math.PI); ctx.stroke()
                }
                if (2 * cp <= range) {
                    ctx.strokeStyle = "rgba(255,214,10,0.6)"
                    ctx.beginPath(); ctx.arc(cx, cy, px(2 * cp), 0, 2 * Math.PI); ctx.stroke()
                }

                // obstacles
                var sideMin = [null, null, null, null]      // front, right, back, left
                var closest = null
                if (!stale) {
                    var d = root._oa.distances
                    var inc = root._oa.increment
                    var off = root._oa.angleOffset
                    var maxcm = root._oa.maxDistance
                    var n = inc > 0 ? Math.min(d.length, Math.round(360 / inc)) : 0
                    for (var i = 0; i < n; i++) {
                        var v = d[i]
                        if (v <= 0 || v >= 65535 || v > maxcm)
                            continue
                        var m = v / 100
                        var ang = ((off + i * inc) % 360 + 360) % 360
                        if (!closest || m < closest[0])
                            closest = [m, ang]
                        var s = Math.floor(((ang + 45) % 360) / 90)
                        if (sideMin[s] === null || m < sideMin[s])
                            sideMin[s] = m
                        var a0 = rad(ang - inc / 2), a1 = rad(ang + inc / 2)
                        if (m <= range) {
                            ctx.fillStyle = shade(m)
                            ctx.beginPath()
                            ctx.arc(cx, cy, px(m), a0, a1)
                            ctx.arc(cx, cy, R, a1, a0, true)
                            ctx.closePath()
                            ctx.fill()
                        }
                        ctx.strokeStyle = col(m)
                        ctx.lineWidth = m <= range ? fs * 0.3 : fs * 0.15
                        ctx.beginPath()
                        ctx.arc(cx, cy, m <= range ? px(m) : R, a0, a1)
                        ctx.stroke()
                    }
                }

                // vehicle icon by type, nose up
                drawVehicle(ctx, cx, cy, fs * 0.95, root.vehicleKind())

                // north marker from the heading
                if (root._vehicle) {
                    var np = pt(-root._vehicle.heading.rawValue, R - fs * 0.15)
                    ctx.fillStyle = "#64b5ff"
                    ctx.beginPath(); ctx.arc(np[0], np[1], fs * 0.42, 0, 2 * Math.PI); ctx.fill()
                    text("N", np[0], np[1] + fs * 0.24, "#0b1a2a", fs * 0.6, true)
                }

                // side readouts
                var letters = ["F", "R", "B", "L"]
                for (var j = 0; j < 4; j++) {
                    var lp = pt(90 * j, R * 0.72)
                    var dm = sideMin[j]
                    var label = letters[j] + " " + (stale ? "--" : (dm === null ? "clear" : dm.toFixed(1)))
                    pill(label, lp[0], lp[1] + fs * 0.25, stale ? "#8a98a6" : (dm === null ? "#30d158" : col(dm)), fs * 0.62)
                }

                // header and footer inside the disc
                text("LIDAR  " + range + " m", cx, cy - R * 0.5, "rgba(200,212,224,0.75)", fs * 0.5, true)
                if (stale) {
                    ctx.fillStyle = "rgba(120,20,16,0.92)"
                    ctx.fillRect(cx - R * 0.62, cy + R * 0.2, R * 1.24, fs * 1.0)
                    text("NO LIDAR DATA", cx, cy + R * 0.2 + fs * 0.72, "#ffffff", fs * 0.62, true)
                } else if (closest) {
                    text(closest[0].toFixed(2) + " m", cx, cy + R * 0.5, col(closest[0]), fs * 0.8, true)
                } else {
                    text("clear", cx, cy + R * 0.5, "#30d158", fs * 0.8, true)
                }
                text(root.ownerName, cx, cy + R * 0.64, "rgba(160,176,192,0.6)", fs * 0.45, false)
            }

            function drawVehicle(ctx, cx, cy, s, kind) {
                ctx.save()
                ctx.translate(cx, cy)
                ctx.strokeStyle = "#e8edf2"
                ctx.fillStyle = "#e8edf2"
                ctx.lineWidth = Math.max(1.5, s * 0.14)
                ctx.lineCap = "round"
                function rotors(n, armDeg0, arm, rr) {
                    for (var k = 0; k < n; k++) {
                        var a = (armDeg0 + k * 360 / n) * Math.PI / 180
                        var x = arm * Math.sin(a), y = -arm * Math.cos(a)
                        ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(x, y); ctx.stroke()
                        ctx.beginPath(); ctx.arc(x, y, rr, 0, 2 * Math.PI); ctx.stroke()
                    }
                    ctx.beginPath(); ctx.arc(0, 0, s * 0.2, 0, 2 * Math.PI); ctx.fill()
                }
                if (kind === "quad")      rotors(4, 45, s * 0.85, s * 0.32)
                else if (kind === "hexa") rotors(6, 30, s * 0.9, s * 0.27)
                else if (kind === "octo") rotors(8, 22.5, s * 0.95, s * 0.22)
                else if (kind === "tri")  rotors(3, 60, s * 0.85, s * 0.32)
                else if (kind === "heli") {
                    ctx.beginPath(); ctx.arc(0, 0, s * 0.95, 0, 2 * Math.PI); ctx.stroke()
                    ctx.beginPath(); ctx.moveTo(0, -s * 0.35); ctx.lineTo(0, s * 1.15); ctx.stroke()
                    ctx.beginPath(); ctx.moveTo(-s * 0.25, s * 1.15); ctx.lineTo(s * 0.25, s * 1.15); ctx.stroke()
                } else if (kind === "plane" || kind === "vtol") {
                    ctx.beginPath(); ctx.moveTo(0, -s * 1.0); ctx.lineTo(0, s * 0.95); ctx.stroke()       // fuselage
                    ctx.beginPath(); ctx.moveTo(-s * 1.1, -s * 0.1); ctx.lineTo(s * 1.1, -s * 0.1); ctx.stroke()  // wing
                    ctx.beginPath(); ctx.moveTo(-s * 0.45, s * 0.85); ctx.lineTo(s * 0.45, s * 0.85); ctx.stroke() // tail
                    if (kind === "vtol") {
                        var pts = [[-s * 0.7, -s * 0.55], [s * 0.7, -s * 0.55], [-s * 0.7, s * 0.4], [s * 0.7, s * 0.4]]
                        ctx.beginPath(); ctx.moveTo(-s * 0.7, -s * 0.55); ctx.lineTo(-s * 0.7, s * 0.4); ctx.stroke()
                        ctx.beginPath(); ctx.moveTo(s * 0.7, -s * 0.55); ctx.lineTo(s * 0.7, s * 0.4); ctx.stroke()
                        for (var p = 0; p < 4; p++) {
                            ctx.beginPath(); ctx.arc(pts[p][0], pts[p][1], s * 0.24, 0, 2 * Math.PI); ctx.stroke()
                        }
                    }
                } else if (kind === "rover") {
                    ctx.strokeRect(-s * 0.45, -s * 0.8, s * 0.9, s * 1.6)
                    var wx = s * 0.62, wy = s * 0.5
                    for (var w = 0; w < 4; w++) {
                        ctx.fillRect((w % 2 ? wx : -wx) - s * 0.1, (w < 2 ? -wy : wy) - s * 0.22, s * 0.2, s * 0.44)
                    }
                } else if (kind === "boat") {
                    ctx.beginPath()
                    ctx.moveTo(0, -s * 1.0); ctx.lineTo(s * 0.45, -s * 0.2); ctx.lineTo(s * 0.4, s * 0.9)
                    ctx.lineTo(-s * 0.4, s * 0.9); ctx.lineTo(-s * 0.45, -s * 0.2); ctx.closePath(); ctx.stroke()
                }
                // flight controller front arrow
                ctx.fillStyle = "#4aa3ff"
                ctx.beginPath()
                ctx.moveTo(0, -s * 1.55); ctx.lineTo(-s * 0.32, -s * 1.08); ctx.lineTo(s * 0.32, -s * 1.08)
                ctx.closePath(); ctx.fill()
                ctx.restore()
            }
        }

        // zoom buttons in the free corners of the square
        Repeater {
            model: [ { label: "\u2212", step: 1, left: true }, { label: "+", step: -1, left: false } ]
            delegate: Rectangle {
                width:          root.buttonSize
                height:         width
                radius:         width / 2
                color:          "#cc1c2228"
                border.color:   "#80a0b0c0"
                anchors.bottom: parent.bottom
                anchors.left:   modelData.left ? parent.left : undefined
                anchors.right:  modelData.left ? undefined : parent.right
                Text {
                    anchors.centerIn:   parent
                    text:               modelData.label
                    color:              "#e8edf2"
                    font.bold:          true
                    font.pixelSize:     root.buttonSize * 0.8
                }
                TapHandler { onTapped: root.zoom(modelData.step) }
            }
        }

        // full screen button (top-right corner)
        Rectangle {
            width:          root.buttonSize
            height:         width
            radius:         width / 2
            color:          root.fullScreen ? "#cc4aa3ff" : "#cc1c2228"
            border.color:   "#80a0b0c0"
            anchors.top:    parent.top
            anchors.right:  parent.right
            Text {
                anchors.centerIn:   parent
                text:               root.fullScreen ? "✕" : "⛶"
                color:              "#e8edf2"
                font.pixelSize:     root.buttonSize * 0.7
            }
            TapHandler { onTapped: root.fullScreen = !root.fullScreen }
        }

        // optical flow panel: height, checks and the no-GPS readiness, above the source selector
        Rectangle {
            id:                         flowPanel
            // left of the radar, top-aligned (above it would cover QGC's gear button)
            anchors.right:              parent.left
            anchors.rightMargin:        root.buttonSize * 0.3
            anchors.top:                parent.top
            width:                      flowCol.implicitWidth + root.buttonSize * 0.6
            height:                     flowCol.implicitHeight + root.buttonSize * 0.4
            radius:                     root.buttonSize * 0.3
            color:                      "#d91c2228"
            border.color:               "#80a0b0c0"
            visible:                    srcBar.visible

            Column {
                id:                 flowCol
                anchors.centerIn:   parent
                spacing:            root.buttonSize * 0.12

                Row {
                    anchors.horizontalCenter:   parent.horizontalCenter
                    spacing:                    root.buttonSize * 0.25
                    Text {
                        anchors.baseline:   hText.baseline
                        text:               "HEIGHT"
                        color:              "#a0b0c0"
                        font.pixelSize:     root.buttonSize * 0.4
                    }
                    Text {
                        id:             hText
                        text:           isNaN(root.flowHeight) ? "--" : root.flowHeight.toFixed(2) + " m"
                        color:          "#e8edf2"
                        font.bold:      true
                        font.pixelSize: root.buttonSize * 0.85
                    }
                }

                Row {
                    anchors.horizontalCenter:   parent.horizontalCenter
                    spacing:                    root.buttonSize * 0.15
                    Repeater {
                        model: [
                            { label: "FLOW", ok: root.srcMode >= 1 && !isNaN(root.flowHeight) },
                            { label: "POS",  ok: root.posOk },
                            { label: "PX4",  ok: root.px4Ready }
                        ]
                        delegate: Rectangle {
                            width:          chk.implicitWidth + root.buttonSize * 0.4
                            height:         root.buttonSize * 0.6
                            radius:         height / 2
                            color:          modelData.ok ? "#cc2fb35a" : "#cc6b2a2a"
                            Text {
                                id:                 chk
                                anchors.centerIn:   parent
                                text:               (modelData.ok ? "✓ " : "✗ ") + modelData.label
                                color:              "#ffffff"
                                font.pixelSize:     root.buttonSize * 0.36
                            }
                        }
                    }
                }

                Rectangle {
                    anchors.horizontalCenter:   parent.horizontalCenter
                    width:                      readyLabel.implicitWidth + root.buttonSize * 0.6
                    height:                     root.buttonSize * 0.75
                    radius:                     height / 2
                    color:                      root.readyColor
                    visible:                    root.readyText !== ""
                    Text {
                        id:                 readyLabel
                        anchors.centerIn:   parent
                        text:               root.readyText
                        color:              "#ffffff"
                        font.bold:          true
                        font.pixelSize:     root.buttonSize * 0.42
                    }
                }
            }
        }

        // position source selector, just above the radar
        Row {
            id:                         srcBar
            anchors.bottom:             parent.top
            anchors.bottomMargin:       root.buttonSize * 0.2
            anchors.horizontalCenter:   parent.horizontalCenter
            spacing:                    root.buttonSize * 0.15
            visible:                    root._vehicle !== null && root._vehicle !== undefined
            Repeater {
                model: [ { label: "GPS", mode: 0 }, { label: "GPS+FLOW", mode: 1 }, { label: "FLOW", mode: 2 } ]
                delegate: Rectangle {
                    property bool active: root.srcMode === modelData.mode
                    width:          srcText.implicitWidth + root.buttonSize * 0.7
                    height:         root.buttonSize * 0.9
                    radius:         height / 2
                    color:          active ? (modelData.mode === 2 ? "#e0ff9f1a" : "#e04aa3ff") : "#cc1c2228"
                    border.color:   active ? "#ffffff" : "#80a0b0c0"
                    Text {
                        id:                 srcText
                        anchors.centerIn:   parent
                        text:               modelData.label
                        color:              "#e8edf2"
                        font.bold:          parent.active
                        font.pixelSize:     root.buttonSize * 0.45
                    }
                    TapHandler { onTapped: root.setSource(modelData.mode) }
                }
            }
        }

        // resize grip (top-left corner): drag up/left = bigger, down/right = smaller
        Rectangle {
            width:          root.buttonSize
            height:         width
            radius:         width / 2
            color:          grip.active ? "#cc4aa3ff" : "#cc1c2228"
            border.color:   "#80a0b0c0"
            anchors.top:    parent.top
            anchors.left:   parent.left
            visible:        !root.fullScreen
            Text {
                anchors.centerIn:   parent
                text:               "⤡"
                color:              "#e8edf2"
                font.pixelSize:     root.buttonSize * 0.75
            }
            HoverHandler { id: gripHover }
            DragHandler {
                id:     grip
                target: null
                property real startScale: 1
                onActiveChanged: if (active) startScale = saved.sizeScale
                onTranslationChanged: {
                    var grow = -(translation.x + translation.y) / 2
                    saved.sizeScale = Math.max(root.minScale, Math.min(root.maxScale,
                                               startScale * (1 + grow / root.radarSize)))
                }
            }
        }
    }
}
