
function out = unifrand(a, b, size)
	out = a + (b-a).*rand(1, size);
end