// retarget.h - Manus glove → robot hand joint retargeting.
//
// Pure functions, no external dependencies. Shared between vmaster and any
// other binary that consumes the same 20-float-per-side glove snapshot.
//
// Caller decides where the destination `q` lives — pass `q+20` for the
// right-hand slot in a q[40] layout, or any other destination.

#pragma once

// H9 - 9 dof: thumb (3) + index/middle/ring (2 each). Side-agnostic.
// Writes 9 floats to `q`.
inline void RetargetSide_h9(const float _q[20], float* q)
{
    q[0] = 1.7f * _q[0]  + 0.85f;
    q[1] = 1.2f * _q[2]  + 0.48f;
    q[2] = 1.2f * _q[3]  + 0.48f;
    q[3] = 1.2f * _q[5]  + 0.0f;
    q[4] = 1.5f * _q[6]  + 0.0f;
    q[5] = 1.2f * _q[9]  + 0.0f;
    q[6] = 1.5f * _q[10] + 0.0f;
    q[7] = 1.2f * _q[13] + 0.0f;
    q[8] = 1.5f * _q[14] + 0.0f;
}

// DG5F - 20 dof, side-dependent ('L' or 'R'). Writes 20 floats to `q`.
inline void RetargetSide_dg5f(char side, const float _q[20], float* q)
{
    if (side == 'L') {
        q[0]  =  1.6f * _q[1]  - 1.5f;
        q[1]  =  1.4f * _q[0]  + 0.5f;
        q[2]  = -1.0f * _q[2]  + 0.0f;
        q[3]  = -1.0f * _q[3]  + 0.0f;

        q[4]  =  1.0f * _q[4]  + 0.2f;
        q[5]  =  1.3f * _q[5]  + 0.0f;
        q[6]  =  1.2f * _q[6]  + 0.0f;
        q[7]  =  1.2f * _q[7]  + 0.0f;
	
        q[8]  =  1.0f * _q[8]  + 0.2f; //+0.2f
        q[9]  =  1.2f * _q[9]  + 0.0f;
        q[10] =  1.2f * _q[10] + 0.0f;
        q[11] =  1.2f * _q[11] + 0.0f;

	q[12] =  1.0f * _q[12] + 0.2f;
        q[13] =  1.2f * _q[13] + 0.0f; //- 0.4f;
        q[14] =  1.2f * _q[14] + 0.0f;
        q[15] =  1.2f * _q[15] + 0.0f;

	//q[16] = -0.1f * _q[17] + 0.0f;
	q[16] =  0.0f * _q[17] + 0.0f;
        q[17] =  1.0f * _q[16] + 0.2f;
        q[18] =  1.0f * _q[17] + 0.0f;
        q[19] =  1.2f * _q[18] + 0.0f;
    } else {
        q[0]  = -1.6f * _q[1]  + 1.5f;
        q[1]  = -1.4f * _q[0]  - 0.5f;
        q[2]  =  1.2f * _q[2]  + 0.0f;
        q[3]  =  1.2f * _q[3]  + 0.0f;
        q[4]  = -1.0f * _q[4]  + 0.2f;
        q[5]  =  1.3f * _q[5]  + 0.0f;
        q[6]  =  1.2f * _q[6]  + 0.0f;
        q[7]  =  1.2f * _q[7]  + 0.0f;
        q[8]  = -1.0f * _q[8]  + 0.0f; //+0.2f
        q[9]  =  1.2f * _q[9]  + 0.0f;
        q[10] =  1.2f * _q[10] + 0.0f;
        q[11] =  1.2f * _q[11] + 0.0f;
        q[12] = -1.0f * _q[12] + 0.0f;
        q[13] =  1.2f * _q[13] - 0.4f;
        q[14] =  1.2f * _q[14] + 0.0f;
        q[15] =  1.2f * _q[15] + 0.0f;
        q[16] =  0.1f * _q[17] + 0.0f;
        q[17] = -1.0f * _q[16] + 0.0f;
        q[18] =  1.0f * _q[17] - 0.4f;
        q[19] =  1.2f * _q[18] + 0.0f;
    }
}
