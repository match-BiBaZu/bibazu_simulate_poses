import copy

original_list = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

# Shallow copy
shallow_copy = original_list

# Deep copy
deep_copy = copy.deepcopy(original_list)

# Modify an element in the shallow copy
shallow_copy[0][0] = 99

print("Original List:", original_list)
print("Shallow Copy:", shallow_copy)
print("Deep Copy:", deep_copy)
