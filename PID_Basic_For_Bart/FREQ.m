% Your system's transfer function
numerator = 1;
denominator = [1, 2, 1];
sys = tf(numerator, denominator);

step(feedback(30 * sys,1))