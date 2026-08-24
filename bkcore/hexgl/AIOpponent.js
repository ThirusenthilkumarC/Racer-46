bkcore.hexgl.waypoints = [[-1319,-124],[-2254,-821],[-2248,-753],[-2246,-683],[-2245,-612],[-2246,-542],[-2246,-472],[-2243,-401],[-2237,-331],[-2224,-261],[-2202,-195],[-2165,-134],[-2115,-82],[-2056,-40],[-1991,-9],[-1924,14],[-1854,31],[-1784,43],[-1714,50],[-1644,55],[-1573,58],[-1503,58],[-1433,57],[-1361,55],[-1289,49],[-1219,28],[-1158,-10],[-1110,-64],[-1076,-127],[-1058,-196],[-1054,-267],[-1061,-337],[-1082,-405],[-1120,-465],[-1174,-512],[-1236,-546],[-1301,-575],[-1364,-605],[-1426,-638],[-1484,-674],[-1541,-715],[-1596,-759],[-1648,-804],[-1697,-853],[-1742,-905],[-1782,-959],[-1818,-1017],[-1847,-1079],[-1868,-1144],[-1879,-1212],[-1880,-1280],[-1868,-1347],[-1844,-1410],[-1808,-1466],[-1762,-1513],[-1706,-1551],[-1645,-1580],[-1579,-1600],[-1511,-1613],[-1442,-1621],[-1373,-1625],[-1304,-1626],[-1233,-1625],[-1164,-1623],[-1095,-1618],[-1026,-1612],[-957,-1604],[-888,-1597],[-817,-1589],[-747,-1580],[-677,-1571],[-608,-1563],[-539,-1556],[-469,-1547],[-399,-1539],[-330,-1533],[-261,-1522],[-194,-1506],[-131,-1483],[-75,-1449],[-30,-1402],[1,-1344],[18,-1279],[23,-1211],[18,-1143],[4,-1075],[-18,-1008],[-45,-944],[-76,-882],[-110,-824],[-149,-768],[-191,-713],[-238,-662],[-287,-613],[-338,-569],[-392,-528],[-450,-490],[-507,-451],[-564,-409],[-619,-364],[-669,-312],[-710,-252],[-741,-187],[-763,-119],[-778,-49],[-786,23],[-787,95],[-783,166],[-772,235],[-757,303],[-736,369],[-708,433],[-675,495],[-634,552],[-588,606],[-537,655],[-482,701],[-423,740],[-359,772],[-292,798],[-222,818],[-150,828],[-78,829],[-7,824],[64,813],[134,796],[202,775],[267,749],[331,719],[391,682],[446,638],[498,588],[544,532],[584,472],[617,409],[643,343],[662,275],[676,205],[686,136],[693,67],[705,1],[729,-61],[767,-114],[818,-154],[880,-177],[946,-182],[1014,-173],[1081,-154],[1144,-127],[1203,-95],[1256,-54],[1306,-4],[1350,50],[1388,106],[1422,166],[1450,229],[1473,294],[1493,360],[1508,429],[1517,497],[1522,566],[1521,636],[1515,705],[1504,773],[1488,840],[1466,905],[1436,968],[1400,1028],[1361,1086],[1316,1138],[1266,1184],[1212,1227],[1154,1265],[1095,1300],[1034,1329],[971,1354],[905,1375],[838,1393],[771,1407],[703,1419],[634,1427],[565,1431],[496,1430],[427,1426],[357,1418],[287,1407],[218,1393],[149,1376],[82,1356],[15,1333],[-52,1308],[-117,1280],[-182,1251],[-245,1221],[-308,1189],[-369,1154],[-431,1120],[-492,1086],[-553,1052],[-612,1017],[-671,981],[-730,945],[-791,909],[-852,874],[-912,840],[-973,806],[-1035,772],[-1099,739],[-1163,709],[-1228,681],[-1293,654],[-1359,631],[-1427,610],[-1497,592],[-1568,577],[-1640,570],[-1712,570],[-1783,578],[-1851,599],[-1914,633],[-1968,680],[-2011,737],[-2046,801],[-2072,867],[-2092,936],[-2108,1005],[-2120,1075],[-2129,1147],[-2135,1218],[-2139,1289],[-2141,1359],[-2141,1430],[-2140,1500],[-2137,1570],[-2131,1641],[-2122,1710],[-2111,1779],[-2098,1849],[-2081,1918],[-2061,1986],[-2037,2053],[-2005,2117],[-1967,2176],[-1923,2231],[-1870,2278],[-1808,2314],[-1739,2334],[-1668,2337],[-1597,2323],[-1531,2296],[-1473,2257],[-1422,2208],[-1377,2154],[-1340,2094],[-1308,2030],[-1281,1966],[-1254,1902],[-1220,1841],[-1179,1786],[-1130,1740],[-1071,1706],[-1007,1685],[-939,1675],[-870,1675],[-800,1683],[-732,1697],[-666,1715],[-601,1736],[-536,1761],[-472,1788],[-408,1818],[-346,1851],[-285,1885],[-226,1920],[-167,1956],[-108,1994],[-49,2031],[12,2066],[73,2100],[136,2132],[200,2159],[267,2182],[335,2203],[404,2217],[473,2225],[544,2230],[615,2232],[685,2231],[754,2229],[823,2226],[892,2221],[961,2214],[1031,2207],[1102,2199],[1171,2189],[1240,2175],[1309,2160],[1376,2141],[1442,2118],[548,1356]];

/*
 * RACER 46 - AI Opponent Controller
 * Handles automated opponent vehicle navigation, physics, heightmap alignment,
 * cornering, checkpoint detection, and lap progress tracking.
 */

var bkcore = bkcore || {};
bkcore.hexgl = bkcore.hexgl || {};

bkcore.hexgl.AIOpponent = function(opts) {
	var self = this;

	this.active = false;
	this.mesh = opts.mesh || null;
	this.dummy = new THREE.Object3D();
	this.dummy.useQuaternion = true;

	this.quaternion = new THREE.Quaternion();
	this.rotation = new THREE.Vector3(0, 0, 0);

	this.waypoints = opts.waypoints || [];
	this.currentWaypointIndex = 0;
	this.targetWaypointIndex = 1;

	this.speed = 0.0;
	this.maxSpeed = opts.maxSpeed || 6.2; // approx 300-320 KM/H
	this.baseSpeed = this.maxSpeed;
	this.acceleration = 0.035;
	this.deceleration = 0.05;
	this.angularSpeed = 0.045;
	this.angularLerp = 0.15;

	this.heightMap = opts.heightMap || null;
	this.heightPixelRatio = opts.heightPixelRatio || (2048.0 / 6000.0);
	this.heightBias = opts.heightBias || 4.0;
	this.heightScale = opts.heightScale || 10.0;
	this.heightLerp = 0.4;

	this.collisionMap = opts.collisionMap || null;
	this.analyser = opts.analyser || null;

	this.lap = 1;
	this.maxLaps = 3;
	this.currentCheckpoint = 0;
	this.previousCheckpoint = -1;
	this.lapTimeElapsed = 0;
	this.lapTimes = [];
	this.finished = false;
	this.finishTime = null;

	this.totalDistance = 0.0;
	this.totalTrackLength = 0.0;
	this.calculateTrackLength();

	if (this.mesh != null) {
		this.dummy.position.copy(this.mesh.position);
	}
};

bkcore.hexgl.AIOpponent.prototype.calculateTrackLength = function() {
	if (!this.waypoints || this.waypoints.length === 0) return;
	var len = 0;
	for (var i = 0; i < this.waypoints.length; i++) {
		var nextIdx = (i + 1) % this.waypoints.length;
		var wp1 = this.waypoints[i];
		var wp2 = this.waypoints[nextIdx];
		len += Math.hypot(wp2[0] - wp1[0], wp2[1] - wp1[1]);
	}
	this.totalTrackLength = len;
};

bkcore.hexgl.AIOpponent.prototype.reset = function(spawnPos, spawnRot) {
	this.active = false;
	this.finished = false;
	this.speed = 0.0;
	this.lap = 1;
	this.currentCheckpoint = 0;
	this.previousCheckpoint = -1;
	this.currentWaypointIndex = 0;
	this.targetWaypointIndex = 1;
	this.lapTimes = [];
	this.finishTime = null;
	this.totalDistance = 0.0;

	if (spawnPos) {
		this.dummy.position.copy(spawnPos);
	}
	if (spawnRot) {
		this.rotation.copy(spawnRot);
		this.quaternion.set(spawnRot.x, spawnRot.y, spawnRot.z, 1).normalize();
		this.dummy.quaternion.copy(this.quaternion);
	}

	if (this.mesh) {
		this.mesh.matrix.identity();
		this.dummy.matrix.setPosition(this.dummy.position);
		this.dummy.matrix.setRotationFromQuaternion(this.dummy.quaternion);
		this.mesh.applyMatrix(this.dummy.matrix);
		this.mesh.updateMatrixWorld(true);
	}
};

bkcore.hexgl.AIOpponent.prototype.update = function(dt) {
	if (!this.active || !this.waypoints || this.waypoints.length === 0) return;

	var pos = this.dummy.position;
	var currTarget = this.waypoints[this.targetWaypointIndex];

	// Calculate distance to current target waypoint
	var dx = currTarget[0] - pos.x;
	var dz = currTarget[1] - pos.z;
	var distToTarget = Math.hypot(dx, dz);

	// Advance to next target waypoint when close
	if (distToTarget < 35) {
		this.currentWaypointIndex = this.targetWaypointIndex;
		this.targetWaypointIndex = (this.targetWaypointIndex + 1) % this.waypoints.length;
		currTarget = this.waypoints[this.targetWaypointIndex];
		dx = currTarget[0] - pos.x;
		dz = currTarget[1] - pos.z;
		distToTarget = Math.hypot(dx, dz);
	}

	// Look-ahead corner intensity detection (check 4 waypoints ahead)
	var lookAheadIndex = (this.targetWaypointIndex + 4) % this.waypoints.length;
	var lookAheadTarget = this.waypoints[lookAheadIndex];
	var laDx = lookAheadTarget[0] - currTarget[0];
	var laDz = lookAheadTarget[1] - currTarget[1];

	var currentAngle = Math.atan2(dx, dz);
	var lookAheadAngle = Math.atan2(laDx, laDz);
	var angleDiff = Math.abs(currentAngle - lookAheadAngle);
	if (angleDiff > Math.PI) angleDiff = Math.PI * 2 - angleDiff;

	// Speed management: slow down for sharp turns, accelerate on straights
	var targetSpeed = this.baseSpeed;
	if (angleDiff > 0.6) {
		targetSpeed *= 0.55; // sharp corner
	} else if (angleDiff > 0.35) {
		targetSpeed *= 0.75; // medium corner
	}

	if (this.speed < targetSpeed) {
		this.speed += this.acceleration * dt;
	} else if (this.speed > targetSpeed) {
		this.speed -= this.deceleration * dt;
	}

	// Smooth steering towards target waypoint
	var targetYaw = Math.atan2(dx, dz);
	var deltaYaw = targetYaw - this.rotation.y;
	while (deltaYaw < -Math.PI) deltaYaw += Math.PI * 2;
	while (deltaYaw > Math.PI) deltaYaw -= Math.PI * 2;

	this.rotation.y += deltaYaw * this.angularLerp * dt;

	// Update rotation quaternion
	this.quaternion.set(0, Math.sin(this.rotation.y / 2), 0, Math.cos(this.rotation.y / 2)).normalize();
	this.dummy.quaternion.copy(this.quaternion);

	// Move forward along facing direction
	var moveDist = this.speed * dt * 15;
	this.dummy.position.x += Math.sin(this.rotation.y) * moveDist;
	this.dummy.position.z += Math.cos(this.rotation.y) * moveDist;

	// Heightmap alignment
	if (this.heightMap && this.heightMap.loaded) {
		var hx = this.heightMap.pixels.width / 2 + this.dummy.position.x * this.heightPixelRatio;
		var hz = this.heightMap.pixels.height / 2 + this.dummy.position.z * this.heightPixelRatio;
		var targetHeight = this.heightMap.getPixelFBilinear(hx, hz) / this.heightScale + this.heightBias;
		if (targetHeight < 16777) {
			this.dummy.position.y += (targetHeight - this.dummy.position.y) * this.heightLerp;
		}
	}

	// Checkpoint & Lap progression tracking
	if (this.analyser && this.analyser.loaded) {
		var cx = Math.round(this.analyser.pixels.width / 2 + this.dummy.position.x * this.heightPixelRatio);
		var cz = Math.round(this.analyser.pixels.height / 2 + this.dummy.position.z * this.heightPixelRatio);
		var col = this.analyser.getPixel(cx, cz);
		if (col.r === 255 && col.g === 255 && col.b < 250) {
			var cp = col.b;
			if (cp === 0 && this.previousCheckpoint === 2) {
				this.previousCheckpoint = cp;
				if (this.lap < this.maxLaps) {
					this.lap++;
				} else {
					this.finished = true;
					this.active = false;
				}
			} else if (cp !== -1 && cp !== this.previousCheckpoint) {
				this.previousCheckpoint = cp;
				this.currentCheckpoint = cp;
			}
		}
	}

	// Calculate total progress distance
	var waypointsPassed = (this.lap - 1) * this.waypoints.length + this.targetWaypointIndex;
	this.totalDistance = waypointsPassed * 80 + (80 - Math.min(80, distToTarget));

	// Synchronize visual mesh with dummy object
	if (this.mesh != null) {
		this.dummy.matrix.setPosition(this.dummy.position);
		this.dummy.matrix.setRotationFromQuaternion(this.dummy.quaternion);
		this.mesh.matrix.identity();
		this.mesh.applyMatrix(this.dummy.matrix);
		this.mesh.updateMatrixWorld(true);
	}
};

bkcore.hexgl.AIOpponent.prototype.getRealSpeed = function() {
	return Math.round(this.speed * 48); // Convert to KM/H equivalent
};

bkcore.hexgl.AIOpponent.prototype.getProgress = function() {
	return this.totalDistance;
};
