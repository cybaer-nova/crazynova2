function quatZ = quatcompress(quat)
%QUATCOMPRESS Summary of this function goes here
%   Detailed explanation goes herestatic inline uint32_t quatcompress(float const q[4])

    if size(quat,1) ~= 4 || norm(quat) ~= 1
        error('Bad quaternion.');
    end

    i_largest = find(abs(quat) == max(abs(quat)),1);

    negate = (quat(i_largest) < 0);

	% do compression using sign bit and 9-bit precision per element.
	quatZ = i_largest;
	for i = 1:4
		if (i ~= i_largest)
			negbit = (quat(i) < 0) ^ negate;
			mag = (bitsll(1,9) - 1) * (abs(quat(i)) / (1/sqrt(2))) + 0.5;
			quatZ = qbitsll(quatZ,10) | qbitsll(negbit,9) | mag;
        end
    end
end



% static inline void quatdecompress(uint32_t comp, float q[4])
% {
% 	unsigned const mask = (1 << 9) - 1;
% 
% 	int const i_largest = comp >> 30;
% 	float sum_squares = 0;
% 	for (int i = 3; i >= 0; --i) {
% 		if (i != i_largest) {
% 			unsigned mag = comp & mask;
% 			unsigned negbit = (comp >> 9) & 0x1;
% 			comp = comp >> 10;
% 			q[i] = ((float)M_SQRT1_2) * ((float)mag) / mask;
% 			if (negbit == 1) {
% 				q[i] = -q[i];
% 			}
% 			sum_squares += q[i] * q[i];
% 		}
% 	}
% 	q[i_largest] = sqrtf(1.0f - sum_squares);
% }