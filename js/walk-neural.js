
var ailoop = null
var NN = null
var KEYMASK = [1,2,4,8]
var nn_ready = false,
    qt_ready = false
var NN_WEIGHT = 10
var QT_WEIGHT = 1
var curState = null
var curAction = 0
var _Q = null
var LIM = {
	'lh_ang':[-1.1,1.1],
	'lh_w':[-4.6,5.1],
	'rh_ang':[-1.1,1.1],
	'rh_w':[-4.1,4.3],
	'lk_ang':[-0.3,1.1],
	'lk_w':[-5.8,4.4],
	'rk_ang':[-0.3,1.1],
	'rk_w':[-6,6.2],
	'T_ang':[-2.1,2.1],
	'T_w':[-2.4,2.8],
	'LL_c':[-1,1],
	'LR_c':[-1,1],
	'LF_y':[5,125],
	'RF_y':[5,125]
}
var ACTIVE_THRESH = 0.1
var ACTIONS = [0,1,2,4,6,8,9]
// function findLimits() {
// 	var state = extractState(body,joint,environment)
// 	for (var ft in state) {
// 		var n = state[ft]
// 		if (n < limits[ft][0]) {
// 			limits[ft][0] = n
// 		}
// 		if (n > limits[ft][1]) {
// 			limits[ft][1] = n
// 		}
// 	}
// }

function stopAI() {
	if (ailoop) {
		clearInterval(ailoop)
        updateKeyState(0);
        ailoop = null
	}
}

function startAI() {
    if (!ailoop) {
        if (qt_ready && nn_ready) {
            run()
        } else {
            loadDatasets()
        }
    }
}

function run() {
	ailoop = setInterval(iterate,50)
}

function iterate() {
    curState = extractState(body,joint,environment)
	curAction = chooseAction(curState)
    updateKeyState(curAction);
}

function chooseAction(s) {
	var bestActions = [],
        bestQ = -Infinity
    var bias = getActionBias(s)
    var k = encodeState(s)
    if (k === 1044668 || k === 1044652) {
        return 8 // yucky hack, I know :p
    }
    for (var i = 0; i < ACTIONS.length; i++) {
        var action = ACTIONS[i]
        var q = QT_WEIGHT*Q(s,action) + NN_WEIGHT*bias[action]
        if (q > bestQ) {
            bestActions = [action]
            bestQ = q
        } else if (q === bestQ) {
            bestActions.push(action)
        }
    }
    return bestActions[floor(bestActions.length*random())]
}

function getActionBias(s) {
    var bias = {0:0,1:0,2:0,4:0,6:0,8:0,9:0};
    var vec = NN.predict(stateVector(s))
    var v0 = vec[0],
    	v1 = vec[1],
    	v2 = vec[2],
    	v3 = vec[3]
    if (v0 > ACTIVE_THRESH) {
    	bias[1]++
    	bias[9]++
    }
    if (v1 > ACTIVE_THRESH) {
    	bias[2]++
    	bias[6]++
    }
    if (v2 > ACTIVE_THRESH) {
    	bias[4]++
    	bias[6]++
    }
    if (v3 > ACTIVE_THRESH) {
    	bias[8]++
    	bias[9]++
    }
    return bias
}

function stateVector(s) {
	var vec = Array(14),
		k = 0
	for (var t in s) {
		vec[k] = round(1000*normalize(s[t],LIM[t]))/1000
		k++
	}
	return vec
}

function actionVector(a) {
	var vec = [0,0,0,0]
	for (var i = KEYMASK.length-1; i >= 0; i--) {
		var k = KEYMASK[i]
		if (a >= k) {
			a -= k
			vec[i] = 1
		}
	}
	return vec
}

function normalize(n,limits) {
	var lo = limits[0],
		hi = limits[1]
	if (n <= lo) {
		return -1
	} else if (n >= hi) {
		return 1
	} else {
		return (2*(n-lo)/(hi-lo))-1
	}
}

function extractState(body,joint,env) {
    return {
        lh_ang: roundToTenThousandth(joint.l_hip.GetJointAngle()),
        lh_w: roundToHundredth(joint.l_hip.GetJointSpeed()),
        rh_ang: roundToTenThousandth(joint.r_hip.GetJointAngle()),
        rh_w: roundToHundredth(joint.r_hip.GetJointSpeed()),
        lk_ang: roundToTenThousandth(joint.l_knee.GetJointAngle()),
        lk_w: roundToHundredth(joint.l_knee.GetJointSpeed()),
        rk_ang: roundToTenThousandth(joint.r_knee.GetJointAngle()),
        rk_w: roundToHundredth(joint.r_knee.GetJointSpeed()),
        T_ang: roundToTenThousandth(body.torso.GetAngle()),
        T_w: roundToHundredth(body.torso.GetAngularVelocity()),
        LL_c: hasContact(body.ll_leg,env.floor) ? 1 : -1,
        LR_c: hasContact(body.lr_leg,env.floor) ? 1 : -1,
        LF_y: roundToTenth(getLeftFootY(body)),
        RF_y: roundToTenth(getRightFootY(body))
    }
}

function loadDatasets(qtable,nnetwork,num_hidden) {
    stopAI()
	qtable = qtable || 'qtab-400'
    nnetwork = nnetwork || 'nnet-14_00' // train0
    num_hidden = num_hidden || 14
    $.getJSON('datasets/'+nnetwork+'.json', function(data){
        NN = new NeuralNetwork(14,num_hidden,4)
        if (data.length) {
            console.log('- Using dataset: '+nnetwork+'.json:');
            console.log('  * Backpropagating '+Object.keys(data).length+' datapoints...');
            for (var i = 0; i < data.length; i++) {
                var t = data[i]
                NN.update(t[0],actionVector(t[1]))
            }
        } else {
            console.log('- Using weights: '+nnetwork+'.json:');
            for (var j = 0; j < num_hidden; j++) {
                NN.hidden[j].bias = data.hidden[j].bias
                NN.hidden[j].weights = data.hidden[j].weights
            }
            for (var k = 0; k < 4; k++) {
                NN.output[k].bias = data.output[k].bias
                NN.output[k].weights = data.output[k].weights
            }
        }
        console.log('...Done!')
        nn_ready = true
        if (nn_ready && qt_ready) {
            run()
        }
    });
    $.getJSON('datasets/'+qtable+'.json', function(data){
        console.log('- Using dataset: '+qtable+'.json:');
        _Q = data
        console.log('  * Loaded '+Object.keys(_Q).length+' states...');
        console.log('...Done!')
        qt_ready = true
        if (nn_ready && qt_ready) {
            run()
        }
    });
}

// Q-table

var features = [];
initFeatures();
var RESOLUTION = 16;
var SQRTRES = sqrt(RESOLUTION);

/**
  Initializes all of the features.
*/
function initFeatures() {

    features.push(function(s) {                      // v0 left knee
        var val = 0.8*(s.lk_ang + 0.25);
        return val;
    });

    features.push(function(s) {                      // v1 right knee
        var val = 0.8*(s.rk_ang + 0.25);
        return val;
    });

    features.push(function(s) {                      // v2 left hip
        var val = 0.5*(s.lh_ang + 1);
        return val;
    });

    features.push(function(s) {                      // v3 right hip
        var val = 0.5*(s.rh_ang + 1);
        return val;
    });

    features.push(function(s) {                      // v4 torso ang
        var val = 0.4*(1.25+max(-1.25,min(1.25,s.T_ang)));
        return val;
    });

    features.push(function(s) {                      // v5 feet heights
        var val = min(max(floor(s.LF_y/20),0),SQRTRES-1);
        val = (val*SQRTRES) + min(max(floor(s.RF_y/20),0),SQRTRES-1);
        return val/RESOLUTION;
    });
}

function Q(s,a) {
    var k = encodeState(s);
    if (k in _Q && a in _Q[k]) {
        return _Q[k][a];
    } else {
        return -1;
    }
}

function encodeState(s) {
    if ('code' in s) {
        return s.code;
    } else {
        var total = 0;
        features.forEach(function(f) {
            var x = discretize(f(s),RESOLUTION);
            total = (RESOLUTION*total) + x;
        });
        s.code = total;
        return total;
    }
}

function discretize(v,n,lb,ub) {
    lb = (lb==undefined) ? 0 : lb;
    ub = (ub==undefined) ? 1 : ub;
    var r = min(n-1,max(0,floor((v-lb)*n)));
    return r;
}