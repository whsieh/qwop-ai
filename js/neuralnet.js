
// For convenience
var exp = Math.exp,
	random = Math.random,
	min = Math.min,
	floor = Math.floor

/**
  Sigmoid transfer function for our neural network
 */
function sigma(x) {
	return 1/(1+exp(-x))
}

/**
  Dot product between two vectors. If they have
  different lengths, use the minimum of the two
  lengths.
*/
function dot(u,v) {
	var k = min(u.length,v.length),
		res = 0
	for (var i = 0; i < k; i++) {
		res += (u[i]*v[i])
	}
	return res
}

/**
  A Neuron represents a node in either the hidden layer
  or the output layer (input 'nodes' are not actually
  saved in this manner).
*/
function Neuron(inputsize) {
	this.weights = Array(inputsize)
	for (var i = 0; i < inputsize; i++) {
		this.weights[i] = random() - 0.5
	}
	this._out = null
	this.bias = random() - 0.5
}

/**
  Given an input vector, computes the activation of Neuron,
  equal to sigma(weights * inputs + bias). This activation
  is bounded between 0 and 1 (due to our sigmoid function)

  If save is some non-null value, the activation for this
  Neuron will also be preserved. This is used by the back-
  propagation algorithm.
*/
Neuron.prototype.activate = function(inputs,save) {
	if (inputs.length !== this.weights.length) {
		throw 'Warning: Incorrect number of inputs (' + inputs.length + ')'
	} else {
		var out = sigma(this.bias + dot(this.weights,inputs))
		this._out = save ? out : null
		return out
	}
}

/**
  Basic 3-layer implementation of a neural network. 'isize'
  indicates the number of inputs to receive; 'hsize' indic-
  ates the size of the hidden layer; 'osize' indicates the
  size of the output layer.

  'alpha' determines momentum and 'eta' determines learning
  rate for backpropagation.
*/
function NeuralNetwork(isize,hsize,osize,decay,alpha,eta) {
	alpha = alpha || 1.0
	eta = eta || 1.0
	decay = decay || 0.99975
	hsize = hsize || floor(2*isize/3)
	osize = osize || 1
	this.alpha = alpha
	this.eta = eta
	this.hidden = Array(hsize)
	for (var i = 0; i < hsize; i++) {
		this.hidden[i] = new Neuron(isize)
	}
	this.output = Array(osize)
	for (var i = 0; i < osize; i++) {
		this.output[i] = new Neuron(hsize)
	}
	this.updatecount = 0
}

/**
  Given a vector of inputs, predicts the output using basic
  feed-forward evaluation. If "save" is non-null, this will
  also save the activations of each hidden and output node
  in the neural network.
*/
NeuralNetwork.prototype.predict = function(inputs,save) {
	var hvals = Array(this.hidden.length),
		ovals = Array(this.output.length)
	for (var j = 0; j < hvals.length; j++) {
		hvals[j] = this.hidden[j].activate(inputs,save)
	}
	for (var k = 0; k < ovals.length; k++) {
		ovals[k] = this.output[k].activate(hvals,save)
	}
	return ovals
}

/**
  Updates the neural network using back-propagation. The
  lengths of "inputs" and "actual" must match the number
  of input nodes and output nodes, respectively.
*/
NeuralNetwork.prototype.update = function(inputs,actual) {
	var alpha = this.alpha,
		eta = this.eta,
		hsize = this.hidden.length,
		osize = this.output.length,
		pred = this.predict(inputs,true),
		output = this.output,
		hidden = this.hidden
	// Compute deltas for output nodes
	var delK = Array(osize)
	for (var k = 0; k < osize; k++) {
		var p = pred[k]
		delK[k] = p*(1-p)*(p-actual[k])
	}
	// Compute deltas for hidden nodes
	var delJ = Array(hsize)
	for (var j = 0; j < hsize; j++) {
		var outN = hidden[j]._out
		var outSum = 0
		for (var k = 0; k < osize; k++) {
			outSum += delK[k]*output[k].weights[j]
		}
		delJ[j] = outN*(1-outN)*outSum
	}
	// Update output node bias
	for (var k = 0; k < output.length; k++) {
		var outnode = output[k],
			tmp = eta*delK[k]
		outnode.bias = alpha*outnode.bias - tmp
		// Update output node weights
		for (var j = 0; j < hsize; j++) {
			var outN = hidden[j]._out
			outnode.weights[j] = alpha*outnode.weights[j] - tmp*outN
		}
	}
	// Update hidden node biases
	for (var j = 0; j < hsize; j++) {
		var delta = delJ[j],
			hnode = hidden[j],
			tmp = eta*delta
		hnode.bias = alpha*hnode.bias - tmp
		for (var i = 0; i < inputs.length; i++) {
			hnode.weights[i] = alpha*hnode.weights[i] - tmp*inputs[i]
		}
	}
	this.updatecount++
	eta *= this.decay
}