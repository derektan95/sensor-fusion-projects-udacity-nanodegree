I followed the following implementation steps for 2D CFAR process as specified on the instructions given in the lecture: 

Determine the number of Training cells for each dimension. Similarly, pick the number of guard cells.
Slide the cell under test across the complete matrix. Make sure the CUT has margin for Training and Guard cells from the edges.
For every iteration sum the signal level within all the training cells. To sum convert the value from logarithmic to linear using db2pow function.
Average the summed values for all of the training cells used. After averaging convert it back to logarithmic using pow2db.
Further add the offset to it to determine the threshold.
Next, compare the signal under CUT against this threshold.
If the CUT level > threshold assign it a value of 1, else equate it to 0.

I selected the training and guard cells such that the final results is a clear visible peak. This means that the guard cells is sufficiently large such that the peak is not factored into the noise calculations. 

The non-thresholded cells at the edge is initialized as 0 in a separate matrix. Only values exceeding threshold were stored into this matrix. (Somewhat an opposite of instructions - but does the job). 
