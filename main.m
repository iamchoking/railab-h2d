x = structout();

syn = struct();
syn.a = 1;
syn.b = 2;

res = subs(x,syn);
disp(res);


% Example matrix
A = [5, 2, 7; 
     1, 9, 4; 
     3, 6, 8];

% Sort the columns based on the values in the first row
[~, idx] = sort(A(1, :)); % Get the indices that would sort the first row
sorted_A = A(:, idx); % Rearrange the columns based on the sorted indices

disp('Original Matrix:');
disp(A);

disp('Matrix with Columns Sorted based on First Row:');
disp(sorted_A);