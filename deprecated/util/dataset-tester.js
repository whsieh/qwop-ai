
var avgJointRotationSpeed = 2.7;
var SELECTED_VAR_LIST = ['LRT_dx','rk_ang','LF_y','lh_ang','lk_lim','lk_ang','LLT_dx','rh_ang',
    'H_y','lh_lim','T_ang','rh_lim','RF_y','LL_c','rk_lim','LR_c'];
var VAR_LIST = ['LRT_dx','rk_w','rk_ang','LF_y','lh_ang','rh_w','lk_lim','lk_ang','LLT_dx','lh_w','rh_ang',
    'H_y','lh_lim','T_ang','rh_lim','lk_w','RF_y','LL_c','rk_lim','T_w','LR_c','H_vy'];

// The algorithm is especially horrible at predicting the following variables.
var BLACKLISTED_VARS = {'T_w':true,'H_vy':true}
var active = false;
var TMData;

$.getJSON('datasets/ds01-results.json', function(response){
    TMData = response;
    console.log('Loaded: ds01-results.json');
});

/**
  Attempts to predict the state 100 ms later
  given the approximate training model calculated
  using the training dataset.
*/
function predictState(s1, a) {

    var s2 = {};
    // For each variable that isn't total s***
    SELECTED_VAR_LIST.forEach(function(v1,i1) {
        // Approximate the value of the variable
        var total = 0;
        VAR_LIST.forEach(function(v2,i2) {
            total += TMData[a][v1][v2] * s1[v2];
        });
        s2[v1] = total;
    });

    var lh_lim = s2['lh_lim'];
    var rh_lim = s2['rh_lim'];
    var lk_lim = s2['lk_lim'];
    var rk_lim = s2['rk_lim'];

    var lk_ang = s2['lk_ang'];
    var rk_ang = s2['rk_ang'];
    var lh_ang = s2['lh_ang'];
    var rh_ang = s2['rh_ang'];

    // Adjust lower leg contact vars (0,1)
    s2['LL_c'] = s2['LL_c'] < 0.5 ? 0 : 1;
    s2['LR_c'] = s2['LR_c'] < 0.5 ? 0 : 1;

    // Adjust left hip limit (-1,0,+1)
    if (lh_lim < -0.5) {
        s2['lh_lim'] = -1;
    } else if (lh_lim > 0.5) {
        s2['lh_lim'] = 1;
    } else {
        s2['lh_lim'] = 0;
    }

    // Adjust right hip limit (-1,0,+1)
    if (rh_lim < -0.5) {
        s2['rh_lim'] = -1;
    } else if (rh_lim > 0.5) {
        s2['rh_lim'] = 1;
    } else {
        s2['rh_lim'] = 0;
    }

    // Adjust left knee limit (-1,0,+1)
    if (lk_lim < -0.5) {
        s2['lk_lim'] = -1;
    } else if (lk_lim > 0.5) {
        s2['lk_lim'] = 1;
    } else {
        s2['lk_lim'] = 0;
    }

    // Adjust right knee limit (-1,0,+1)
    if (rk_lim < -0.5) {
        s2['rk_lim'] = -1;
    } else if (rk_lim > 0.5) {
        s2['rk_lim'] = 1;
    } else {
        s2['rk_lim'] = 0;
    }

    // Adjust all knee angles to fit within box2d's limits
    s2['lk_ang'] = lk_ang > kneeLimits[1] ? kneeLimits[1] : lk_ang;
    s2['lk_ang'] = lk_ang < kneeLimits[0] ? kneeLimits[0] : lk_ang;
    s2['rk_ang'] = rk_ang > kneeLimits[1] ? kneeLimits[1] : rk_ang;
    s2['rk_ang'] = rk_ang < kneeLimits[0] ? kneeLimits[0] : rk_ang;

    // Adjust all hip angles to fit within box2d's limits
    s2['lh_ang'] = lh_ang > hipLimits[1] ? hipLimits[1] : lh_ang;
    s2['lh_ang'] = lh_ang < hipLimits[0] ? hipLimits[0] : lh_ang;
    s2['rh_ang'] = rh_ang > hipLimits[1] ? hipLimits[1] : rh_ang;
    s2['rh_ang'] = rh_ang < hipLimits[0] ? hipLimits[0] : rh_ang;

    // Determine ang. vel. for knees and hips
    switch(a) {
        case 0:
        s2['lk_w'] = 0;
        s2['rk_w'] = 0;
        s2['lh_w'] = 0;
        s2['rh_w'] = 0;
        break;
        case 1:
        s2['lk_w'] = 0;
        s2['rk_w'] = 0;
        s2['lh_w'] = avgJointRotationSpeed;
        s2['rh_w'] = -avgJointRotationSpeed;
        break;
        case 2:
        s2['lk_w'] = 0;
        s2['rk_w'] = 0;
        s2['lh_w'] = -avgJointRotationSpeed;
        s2['rh_w'] = avgJointRotationSpeed;
        break;
        case 4:
        s2['lk_w'] = avgJointRotationSpeed;
        s2['rk_w'] = -avgJointRotationSpeed;
        s2['lh_w'] = 0;
        s2['rh_w'] = 0;
        break;
        case 6:
        s2['lk_w'] = avgJointRotationSpeed;
        s2['rk_w'] = -avgJointRotationSpeed;
        s2['lh_w'] = -avgJointRotationSpeed;
        s2['rh_w'] = avgJointRotationSpeed;
        break;
        case 8:
        s2['lk_w'] = -avgJointRotationSpeed;
        s2['rk_w'] = avgJointRotationSpeed;
        s2['lh_w'] = 0;
        s2['rh_w'] = 0;
        break;
        case 9:
        s2['lk_w'] = -avgJointRotationSpeed;
        s2['rk_w'] = avgJointRotationSpeed;
        s2['lh_w'] = avgJointRotationSpeed;
        s2['rh_w'] = -avgJointRotationSpeed;
        break;
        default:
        s2['lk_w'] = 0;
        s2['rk_w'] = 0;
        s2['lh_w'] = 0;
        s2['rh_w'] = 0;
        break;
    }

    // All hope is lost on these two...
    s2['T_w'] = s1['T_w'];
    s2['H_vy'] = s1['H_vy'];

    return s2;
}

function testApproximateTransition() {
    
    var predictedState = predictState(extractState(),keyState);
    var oldKeyState = keyState;

    setTimeout(function() {
        if (active && oldKeyState == keyState) {
            var stateDiff = {'a' : keyState};
            var curState = extractState();       
            VAR_LIST.forEach(function(v,i) {
                if (!(v in BLACKLISTED_VARS)) {
                    var variableDiff = {};
                    var actual = curState[v];
                    var predicted = predictedState[v];
                    var error = Math.round(100*(actual - predicted)/actual);
                    variableDiff['actual'] = actual;
                    variableDiff['predicted'] = predicted;
                    variableDiff['error'] = error;
                    stateDiff[v] = (variableDiff);
                }
            });
            $('body').append(JSON.stringify(stateDiff) + ',');
        }
    },100);
}

function extractState() {
    return {
        lh_ang: roundToTenThousandth(joint.l_hip.GetJointAngle()),
        lh_w: roundToHundredth(joint.l_hip.GetJointSpeed()),
        lh_lim: hipAtLimit(joint.l_hip),

        rh_ang: roundToTenThousandth(joint.r_hip.GetJointAngle()),
        rh_w: roundToHundredth(joint.r_hip.GetJointSpeed()),
        rh_lim: hipAtLimit(joint.r_hip),

        lk_ang: roundToTenThousandth(joint.l_knee.GetJointAngle()),
        lk_w: roundToHundredth(joint.l_knee.GetJointSpeed()),
        lk_lim: kneeAtLimit(joint.l_knee),

        rk_ang: roundToTenThousandth(joint.r_knee.GetJointAngle()),
        rk_w: roundToHundredth(joint.r_knee.GetJointSpeed()),
        rk_lim: kneeAtLimit(joint.r_knee),

        T_ang: roundToTenThousandth(body.torso.GetAngle()),
        T_w: roundToHundredth(body.torso.GetAngularVelocity()),
        H_y: roundToTenth(body.head.GetPosition().y),
        H_vy: roundToHundredth(body.head.GetLinearVelocity().y),
        LLT_dx: roundToTenth(body.ll_leg.GetPosition().x - body.torso.GetPosition().x),
        LRT_dx: roundToTenth(body.lr_leg.GetPosition().x - body.torso.GetPosition().x),
        LF_y: roundToTenth(getLeftFootHeight()),
        RF_y: roundToTenth(getRightFootHeight()),
        LL_c: hasContact(body.ll_leg,environment.floor) ? 1 : 0,
        LR_c: hasContact(body.lr_leg,environment.floor) ? 1 : 0
    };
}

function roundToTenThousandth(n) {
    return Math.round(n * 10000) / 10000;
}

function roundToHundredth(n) {
    return Math.round(n * 100) / 100;
}

function roundToTenth(n) {
    return Math.round(n * 10) / 10;
}

function beginTest() {
    $('body').append('<br> [');
    if (!active) {
        setInterval(testApproximateTransition,500);
    }
    active = true;
}

function endTest() {
    $('body').append(']');
    active = false;
}