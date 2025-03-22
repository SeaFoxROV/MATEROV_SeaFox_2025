
menos = 0.0027397260273
sum = 1
im = 1
for i in range(23):
    sum *= im-menos
    im -= menos
print(sum)