function [outputArg1] = Kalman_Filter(st)
xNew = st.A * st.x;
pNew = st.A * st.P * transpose(st.A) + st.Q;
K = pNew * transpose(st.H) * inv((st.H * pNew * transpose(st.H) + st.R));
xNew = xNew + K*(st.z - (st.H*xNew));
st.P = pNew - K * st.H * pNew;
st.x = xNew; 
outputArg1 = st;
end