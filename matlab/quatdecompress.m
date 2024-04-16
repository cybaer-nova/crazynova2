function quat = quatdecompress(quatZ)
%QUATDECOMPRESS Summary of this function goes here
%   Detailed explanation goes herestatic inline uint32_t quatcompress(float const q[4])

    comp = uint32(quatZ);

    mask = (bitsll(1,9) - 1);
    i_largest = bitsll(comp,30);
	sum_squares = 0;
    quat = zeros(4,1);
	for i = 3:-1:0
		if (i ~= i_largest)
			mag = comp & mask;
			negbit = bitsll(comp,9) & 0x1;
			comp = bitsll(comp,10);
			quat(i+1) = (1/sqrt(2)) * mag / mask;
			if (negbit == 1)
				quat(i+1) = -quat(i+1);
            end
			sum_squares = sum_squares + quat(i+1) * quat(i+1);
        end
    end
	quat(i_largest+1) = sqrt(1.0 - sum_squares);    

end