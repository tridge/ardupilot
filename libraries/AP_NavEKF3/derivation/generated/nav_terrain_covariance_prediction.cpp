// Equations for covariance matrix prediction, without process noise!
const ftype PS0 = P[0][0]*dt + P[0][2];
const ftype PS1 = P[0][1]*dt;
const ftype PS2 = PS1 + P[1][2];
const ftype PS3 = P[1][1]*dt + P[1][3];


nextP[0][0] = P[0][0];
nextP[0][1] = P[0][1];
nextP[1][1] = P[1][1];
nextP[0][2] = PS0;
nextP[1][2] = PS2;
nextP[2][2] = PS0*dt + P[0][2]*dt + P[2][2];
nextP[0][3] = PS1 + P[0][3];
nextP[1][3] = PS3;
nextP[2][3] = PS2*dt + P[0][3]*dt + P[2][3];
nextP[3][3] = PS3*dt + P[1][3]*dt + P[3][3];
nextP[0][4] = P[0][4];
nextP[1][4] = P[1][4];
nextP[2][4] = P[0][4]*dt + P[2][4];
nextP[3][4] = P[1][4]*dt + P[3][4];
nextP[4][4] = P[4][4];


