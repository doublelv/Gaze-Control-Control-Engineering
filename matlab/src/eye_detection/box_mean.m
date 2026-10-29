function B = box_mean(G, w)
%BOX_MEAN  Local mean over a w x w window (edges normalised), base MATLAB.
w = max(1, round(w));
k = ones(w, 1)/w;
num = conv2(conv2(G, k, 'same'), k', 'same');
den = conv2(conv2(ones(size(G)), k, 'same'), k', 'same');
B = num./den;
end
