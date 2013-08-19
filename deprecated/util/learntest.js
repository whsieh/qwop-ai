
var canv = document.getElementById('canvas');
var ctx = canv.getContext('2d');
var cWidth = canv.width = Math.min(document.body.scrollWidth,document.body.scrollHeight);
var cHeight = canv.height = cWidth;

var MOVES = {
	'N':[0,-1],'E':[1,0],'W':[-1,0],'S':[0,1]
};
var EMPTY = 0, PLAYER = 1, ENEMY = 2, TARGET = 3;
var	grid = [[1,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],
			[0,0,0,2,0,2],[0,0,0,0,0,0],[0,0,0,2,0,3]];
var OFFSET = 50;

var gamestate = 0;
var LOSS = -1;
var WIN = 1;
var ONGOING = 0;

var LOCATIONS = {
	'P':[0,0],
	'E1':[3,3],
	'E2':[3,5],
	'E3':[5,3],
	'T':[5,5],
}

var alpha = 0.9, gamma = 0.5;

var feats = [], weights = [];

var prevState = undefined;
var prevAction = undefined;
var curState = undefined;
var curAction = undefined;
var prevReward = undefined;

function manh_dist(pos1, pos2) {
	return Math.abs(pos1[0] - pos2[0]) +
	 Math.abs(pos1[1] - pos2[1]);
}

function init_features_weights() {

	// Inverse manhattan distance to the nearest enemy.
	feats.push(function(s,a) {
		var newI = s['P'][0];
		var newJ = s['P'][1];
		if (a !== null) {
			newI += MOVES[a][0];
			newJ += MOVES[a][1];
		}
		var mindist = Math.min(
			manh_dist([newI,newJ],s['E1']),
			manh_dist([newI,newJ],s['E2']),
			manh_dist([newI,newJ],s['E3']));
		return 1/(0.1+mindist);
	});
	weights.push(0);

	// Average inverse manhattan distance to enemies.
	feats.push(function(s,a) {
		var newI = s['P'][0];
		var newJ = s['P'][1];
		if (a !== null) {
			newI += MOVES[a][0];
			newJ += MOVES[a][1];
		}
		var e1 = 1/(0.1+manh_dist([newI,newJ],s['E1']))
		var e2 = 1/(0.1+manh_dist([newI,newJ],s['E2']))
		var e3 = 1/(0.1+manh_dist([newI,newJ],s['E3']))
		return (e1 + e2 + e3) / 3;
	});
	weights.push(0);

	// Inverse manhattan distance to the target.
	feats.push(function(s,a) {
		var newI = s['P'][0];
		var newJ = s['P'][1];
		if (a !== null) {
			newI += MOVES[a][0];
			newJ += MOVES[a][1];
		}
		var dist = manh_dist([newI,newJ],s['T']);
		return 1/(0.1+dist);
	});
	weights.push(0);
}

function Q(s,a) {
	var total = 0;
	for (var i in feats) {
		total += weights[i]*feats[i](s,a);
	}
	return total;
}

function update(s1,a,r,s2) {
	var d = diff(s1,a,r,s2);
	for (var i in weights) {
		var f = feats[i](s1,a);
		weights[i] += alpha*d*f;
		if (i == 2 && weights[i] < 0) {
			weights[i] = 0;
		}
	}
}

function diff(s1,a,r,s2) {
	var maxQ = -Infinity;
	lmoves = legal_moves(s2,s2['P'][0],s2['P'][1]);
	if (lmoves.length > 0) {
		lmoves.forEach(function(a2) {
			var q = Q(s2,a2);
			if (q > maxQ) {
				maxQ = q;
			}
		});
	} else {
		maxQ = Q(s2,null);
	}
	return r + (gamma*maxQ) - Q(s1,a);
}

function choose_move(s2) {
	var maxQ = -Infinity;
	var best = null;
	lmoves = legal_moves(s2,s2['P'][0],s2['P'][1]);
	if (lmoves.length > 0) {
		lmoves.forEach(function(a2) {
			var q = Q(s2,a2);
			if (q > maxQ) {
				maxQ = q;
				best = a2;
			}
		});
	}
	return best;
}

/* ======================================================= */

function run_game(timestep) {

	timestep = (timestep==undefined) ? 1000 : timestep;

	var game = setInterval(function() {
		draw_grid();
		if (gamestate !== 0) {
			reset(timestep)
		} else {
			gamestate = step();
		}
	},timestep);
}

function step(action) {

	var status = ONGOING;

	prevState = curState;
	prevAction = curAction;

	curState = copy(LOCATIONS)
	curAction = choose_move(curState);

	for (var agent in LOCATIONS) {
		var pos = LOCATIONS[agent];
		var lmoves = legal_moves(LOCATIONS,pos[0], pos[1]);

		// console.log('From ' + pos + ' you can take actions: ' + lmoves)

		var m = null;

		var type = grid[pos[0]][pos[1]];

		switch(type) {

			case 1: 	// Player
            m = curAction;
			break;

			case 2: 	// Enemy
			m = lmoves[Math.floor(Math.random()*lmoves.length)];
			break;

			case 3: 	// Target
			m = lmoves[Math.floor(Math.random()*lmoves.length)];
			break;
		}

		if (m !== null && lmoves.length > 0) {
			
			// console.log('    I am taking: ' + m);
			// console.log('    This means: ' + MOVES[m]);

			var newI = pos[0]+MOVES[m][0];
			var newJ = pos[1]+MOVES[m][1];

			// console.log('    I have traveled from ' + pos + ' to ' + [newI,newJ]);

			if (type == ENEMY && grid[newI][newJ] == PLAYER) {
				console.log('THE PLAYER HAS BEEN DEFEATED.');
				status = LOSS;
			}

			if (type == PLAYER && grid[newI][newJ] == TARGET) {
				console.log('THE PLAYER HAS REACHED THE TARGET.');
				status = WIN;
			}

			grid[newI][newJ] = type;
			grid[pos[0]][pos[1]] = 0;
			// console.log('  Moving ' + agent + ' to coordinates: ' + [newI, newJ]);
			LOCATIONS[agent] = [newI,newJ];
		}
	}

	if (prevState !== undefined && prevAction !== undefined) {
		var reward = (status==1) ? 50 : ((status==-1) ? -10 : 0);
		update(prevState, prevAction, 10*status, curState);
	}

	return status;
}

function reset(timestep) {

	gamestate = 0;
	LOCATIONS = {'P':[0,0],'E1':[3,3],'E2':[3,5],'E3':[5,3],'T':[5,5]};
	grid = [[1,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],
			[0,0,0,2,0,2],[0,0,0,0,0,0],[0,0,0,2,0,3]];
	prevState = undefined;
	prevAction = undefined;
	curState = undefined;
	curAction = undefined;
	prevReward = undefined;
}

function draw_grid() {

	ctx.clearRect(0,0,cWidth,cHeight);
	for (var i in grid) {
		for (var j in grid[i]) {

			var xStart = 100*i + OFFSET, yStart = 100*j + OFFSET;
			_draw_box(xStart,yStart,100);

			switch(grid[i][j]) {

				case 1: 	// Player
                _draw_circle(xStart+50, yStart+50, 30, '#0000FF');
				break;

				case 2: 	// Enemy
				_draw_circle(xStart+50, yStart+50, 30, '#FF0000');
				break;

				case 3: 	// Target
				_draw_circle(xStart+50, yStart+50, 30, '#00FF00');
				break;

			}
		}
	}
}

function _draw_circle(xC,yC,radius,color) {
	ctx.beginPath();
    ctx.arc(xC, yC, radius, 2 * Math.PI, false);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.lineWidth = 5;
    ctx.strokeStyle = '#003300';
    ctx.stroke();
}

function _draw_box(xStart,yStart,side) {
	ctx.beginPath();
	ctx.moveTo(xStart,yStart);
	ctx.lineTo(xStart + side, yStart);
	ctx.lineTo(xStart + side, yStart + side);
	ctx.lineTo(xStart, yStart + side);
	ctx.lineTo(xStart,yStart);
    ctx.lineWidth = 5;
    ctx.strokeStyle = '#003300';
    ctx.stroke();
}

function copy(obj) {
	return jQuery.extend(true, {}, obj);
}

function legal_moves(s,i,j) {

	var m = [];

	for (var dir in MOVES) {
		var newI = i + MOVES[dir][0], newJ = j + MOVES[dir][1];
		if (newI >= 0 && newI < grid.length &&
			newJ >= 0 && newJ < grid.length) {
			if (posEqual(s['P'],[i,j])) {
				if (posEqual(s['T'],[newI,newJ])) {
					m.push(dir);
				}
			}
			if (posEqual(s['E1'],[i,j]) || posEqual(s['E2'],[i,j])
				|| posEqual(s['E3'],[i,j])) {
				if (posEqual(s['P'],[newI,newJ])) {
					m.push(dir);
				}
			}
			var blocked = false;
			for (var agt in s) {
				if (posEqual(s[agt],[newI,newJ])) {
					blocked = true;
					break;
				}
			}
			if (!blocked) {
				m.push(dir)
			}
		}
	}

	return m;
}

init_features_weights();

/**
	MISCELLANEOUS UTILITY FUNCTIONS
*/
function posEqual(a, b) {
	return (a[0]==b[0]) && (a[1]==b[1]);
}