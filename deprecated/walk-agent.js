
var iterations = 0
var nextAction = 0
var active = false;

var stepBegin = false;
var stepHalf = false;
var stepComplete = false;
var stepScore = 0;
var hasTraveled = false;
var hasTraveledBack = false;
var fallen = false;
var paused = false;
var stepcount = 0;

var prevState = undefined;
var curState = undefined;
var prevAction = 0;
var curAction = 0;
var prevR = 0;
var curR = 0;
var prevExpState = undefined;
var curExpState = undefined;

var sampleCount = 0;
var pctErr = 0;

var VAR_LIST = ['lk_ang','rk_ang','lh_ang','rh_ang','T_ang','LF_y','RF_y'];

var lastStateVec = [0,0,0,0,0,0];
var inactionCounter = 0;
var RESET_THRESH = 8;

/* Highly experimental methods. Ugh... */
// var qTrees =    {0: new kdTree([],stateDist,features.length),
//                 1: new kdTree([],stateDist,features.length),
//                 2: new kdTree([],stateDist,features.length),
//                 4: new kdTree([],stateDist,features.length),
//                 6: new kdTree([],stateDist,features.length),
//                 8: new kdTree([],stateDist,features.length),
//                 9: new kdTree([],stateDist,features.length)};
// var qTreeSizes = {0:0,1:0,2:0,4:0,6:0,8:0,9:0};

// function updateQTree(k,a) {
//     qTrees[a].insert(asVector(k));
//     qTreeSizes[a]++;
// }

// function searchQTree(k,a) {
//     var res = qTrees[a].nearest(asVector(k),25,10);
//     res.forEach(function(data,index) {
//         res[index][0] = _Q[asNumber(data[0])][a];
//     });
//     return res;
// }

// function loadQTrees() {
//     $.getJSON('datasets/qn07.json', function(data){
//         console.log('- Using dataset: qn07.json:');
//         _Q = data['Q'];
//         console.log('  * loaded '+Object.keys(_Q).length+' samples into Q.');
//         console.log('  - loading QTree:');
//         var i = 1,x = 0;
//         for (k in _Q) {
//             for (a in _Q[k]) {
//                 // Only store ~10% of total samples in the tree.
//                 if (i % 10 == 0) {
//                     updateQTree(k,a);
//                     qTreeSizes[a]++;
//                     x++;
//                     if (x % 1000 == 0) {
//                         console.log('    ('+x+') datapoints.');
//                     }
//                 }
//                 i++;
//             }
//         }
//         for (var a in qTreeSizes) {
//             console.log('  * loaded '+qTreeSizes[a]+' samples into q-Tree['+a+']');
//         }
//         console.log('...Done!');
//         // loadWalkData('walk');
//     });
// }

function loadDatasets(fname) {
    fname = fname==undefined ? 'qn07' : fname;
    $.getJSON('datasets/'+fname+'.json', function(data){
        console.log('- Using dataset: '+fname+'.json:');
        _Q = data['Q'];
        console.log('  * loaded '+Object.keys(_Q).length+' samples into Q.');
        _N = data['N'];
        console.log('  * loaded '+Object.keys(_N).length+' samples into N.');
        // console.log('  - loading QTree:');
        // var i = 1,x = 0;
        // for (k in _Q) {
        //     for (a in _Q[k]) {
        //         // Only store ~10% of total samples in the tree.
        //         if (i % 10 == 0) {
        //             updateQTree(k,a);
        //             QTreeSizes[a]++;
        //             x++;
        //             if (x % 1000 == 0) {
        //                 console.log('    ('+x+') datapoints.');
        //             }
        //         }
        //         i++;
        //     }
        // }
        // for (var a in QTreeSizes) {
        //     console.log('  * loaded '+QTreeSizes[a]+' samples into QTree['+a+']');
        // }
        console.log('...Done!')
    });
}

function loadWalkData(fname) {
    fname = fname==undefined ? 'walk0' : fname;
    $.getJSON('datasets/'+fname+'.json',function(data) {
        console.log('- Loading walk data from ' + fname + '.json:')
        var count = 0;
        data.forEach(function(d) {
            var stepdata = d['data'];
            for (k in stepdata) {
                walkTree.insert(asVector(k));
                walkActions[k] = stepdata[k];
                count++;
                if (count % 500 == 0) {
                    console.log('  ...' + count);
                }
            }
        });
        console.log('Done: added ' + count + ' states.');
    });
}

function extractState(body,joint,environment,angles,prevAction) {

    // var cm = center_of_mass(body);
    // var headVelX = body.head.GetLinearVelocity();

    if (!angles) {
        var lh = roundToTenThousandth(joint.l_hip.GetJointAngle());
        var rh = roundToTenThousandth(joint.r_hip.GetJointAngle());
        var lk = roundToTenThousandth(joint.l_knee.GetJointAngle());
        var rk = roundToTenThousandth(joint.r_knee.GetJointAngle());
    } else {
        var lh = angles.lh;
        var rh = angles.rh;
        var lk = angles.lk;
        var rk = angles.rk;
    }

    return {

        lh_ang: lh,
        // lh_w: roundToHundredth(joint.l_hip.GetJointSpeed()),

        rh_ang: rh,
        // rh_w: roundToHundredth(joint.r_hip.GetJointSpeed()),

        lk_ang: lk,
        // lk_w: roundToHundredth(joint.l_knee.GetJointSpeed()),

        rk_ang: rk,
        // rk_w: roundToHundredth(joint.r_knee.GetJointSpeed()),

        T_ang: roundToTenThousandth(body.torso.GetAngle()),
        // T_w: roundToHundredth(body.torso.GetAngularVelocity()),
        // T_x: body.torso.GetPosition().x,
        // T_vx: roundToHundredth(body.torso.GetLinearVelocity().x),

        // H_y: roundToTenth(body.head.GetPosition().y),
        // H_vy: roundToHundredth(headVelX.y),
        // H_vx: roundToHundredth(headVelX.x),

        // LL_x: body.ll_leg.GetPosition().x,
        // LR_x: body.lr_leg.GetPosition().x,
        // LL_vx: roundToHundredth(body.ll_leg.GetLinearVelocity().x),
        // LR_vx: roundToHundredth(body.lr_leg.GetLinearVelocity().x),
        // LL_c: hasContact(body.ll_leg,environment.floor) ? 1 : 0,
        // LR_c: hasContact(body.lr_leg,environment.floor) ? 1 : 0,

        // UL_x: body.ul_leg.GetPosition().x,
        // UR_x: body.ur_leg.GetPosition().x,

        // LF_x: roundToTenth(getLeftFootX(body)),
        LF_y: roundToTenth(getLeftFootY(body)),
        // RF_x: roundToTenth(getRightFootX(body)),
        RF_y: roundToTenth(getRightFootY(body)),
        pA: prevAction,

        // CM_x: roundToTenth(cm.x),
        // CM_y: roundToTenth(cm.y),

        'body':body,
        'joint':joint,
        'env':environment
    };
}

function notifyDistTraveled() {
    hasTraveled = true;
}

function notifyBackwardsTraveled() {
    hasTraveledBack = true;
}

function notifyStepBegun() {
    stepBegin = true;
}

function notifyHalfStep() {
    stepHalf = true;
}

function notifyStepComplete(distance) {
    stepComplete = true;
    stepScore = distance;
}

function notifyFall(steps) {
    fallen = true;
    stepcount = steps;
}

function setOutput(b) {
    output = b;
}

function _calculate_penalty() {

    var penalty = 0

    if (fallen) {
        var fallPenalty = 30;
        if (body.torso.GetAngle() > 1.5) {
            // Because BUZZBOMB
            // ..actually, because falling backwards is bad.
            fallPenalty += 20;
        }
        if (output) {
            console.log('(-) Fall detected. -' + fallPenalty + ' points.');
        }
        fallen = false;
        penalty -= fallPenalty;
    }
    if (hasTraveledBack) {
        if (output) {
            console.log('(-) Traveled backwards. -25 points.');
        }
        hasTraveledBack = false;
        penalty -= 25;
    }
    if (totalDistTraveled < -5) {
        if (output) {
            console.log('(-) Why are you even here?! -100 points.');
        }
        penalty -= 100;
    }
    if (body.ll_leg.GetPosition().y < 15) {
        if (output) {
            console.log('(-) Your left leg is sinking! -50 points.');
        }
        penalty -= 50;
    }
    if (body.lr_leg.GetPosition().y < 15) {
        if (output) {
            console.log('(-) Your right leg is sinking! -50 points.');
        }
        penalty -= 50;
    }
    return penalty;
}

function _calculate_bonus() {

    var reward = 0;

    if (stepBegin) {
        stepBegin = false;
        if (output) {
            console.log('(+) Step begun. +2 points.');
        }
        reward += 2;
    } else if (stepHalf) {
        stepHalf = false;
        if (output) {
            console.log('(+) Step half finished. +5 points.');
        }
        reward += 5;
    } else if (stepComplete) {
        stepComplete = false;
        var stepreward = (stepScore > 40) ? 6*stepScore : max(0,3*stepScore)
        stepreward = round(stepreward + 5);
        if (output) {
            console.log('(+) STEP COMPLETE! ' + stepreward + ' points.');
        }
        reward += stepreward;
    }

    if (hasTraveled) {
        hasTraveled = false;
        if (output) {
            console.log('(+) You have traveled 2 meters! +2 points.');
        }
        reward += 2;
    }
    return reward;
}

function getScore() {
    return _calculate_bonus() + _calculate_penalty() - 1; // -1 action bias
}

function calculate_Q_scores(prevA) {
    var s = extractState(body,joint,environment,null,prevA);
    ACTIONS.forEach( function(action) {
        qscores[action] = Q(s,action);
    });
}

function alphaDecay(a) {
    return a*0.9997;
}

function iterate_active() {

    if (!paused) {

        if (iterations % 100 == 0 && iterations > 0) {
            console.log('(~) Iteration count: ' + iterations);
            if (alpha > 0.1) {
                alpha = alphaDecay(alpha);
            }
        }

        prevState = curState;
        prevAction = curAction;

        // if (prevState !== undefined) {
        //     var p = predictWorld(prevState['body'],prevState['joint'],prevAction,prevState['pA']);
        //     var predictedState = extractState(p['body'],p['joint'],p['env'],p['angles']);
        //     fakeWorld = p.world;
        // }

        curState = extractState(body,joint,environment,null,prevAction);
        curAction = chooseAction(curState);
        // curAction = keyState;

        prevR = curR;
        curR = getScore();

        if (prevState !== undefined && prevAction !== undefined) {
            update(prevState, prevAction, curR, curState);
            updateN(curState,curAction);
            iterations++;

            // logPredictionError(curState,predictedState);
        }
        updateKeyState(curAction);

        var curStateVec = asVector(encodeState(curState))
        var sDiff = stateDist(lastStateVec,curStateVec)
        if (sDiff > RESET_THRESH) {
            lastStateVec = curStateVec;
            inactionCounter = 0;
        } else {
            if (inactionCounter > 50) {
                inactionCounter = 0;
                resetRunner();
            } else {
                inactionCounter++;
            }
        }
    }

    calculate_Q_scores(prevAction);
}

function chooseActionFromLog() {
    if (keyStateLog.length > 0) {
        console.log(keyStateLog[0])
        if (keyStateLog[0]['duration'] == 1) {
            var s = keyStateLog.shift();
            return s['action'];
        } else {
            keyStateLog[0]['duration']--;
            return keyStateLog[0]['action'];
        }
    } else {
        return 0;
    }
}

function advance_step() {
    run(timePerActionMs);
}

function run(time) {
    if (!active) {
        active = true;
        console.log('Beginning reinforcement learning...')
    	var runProcess = setInterval(
    		iterate_active,
    		timePerActionMs
    	);
    } else {
        if (time==undefined) {
            paused = false;
            mainLoopPaused = false;
        } else {
            paused = false;
            mainLoopPaused = false;
            setTimeout(function() {
                pause();
            },time);
        }
    }
}

function writeQTable() {
    $('body').append(JSON.stringify({'Q':_Q,'N':_N}));
}

function pause() {
    paused = true;
    mainLoopPaused = true;
}

function logPredictionError(s2,s2_pred) {
    
    VAR_LIST.forEach(function(v,i) {
        var actual = s2[v];
        var predicted = s2_pred[v];
        var error = Math.round(100*(actual - predicted)/actual);
        var_act[i] = roundToTenThousandth(actual);
        var_pre[i] = roundToTenThousandth(predicted);
        var_err[i] = error;
    });
}
