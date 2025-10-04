import numbers

def fibonacci(n):
    if not isinstance(n, numbers.Number) or not isinstance(n, int):
        raise TypeError("Input must be an integer.")
    if n < 0:
        raise ValueError("Input must be a non-negative integer.")
    elif n == 0:
        return 0
    elif n == 1:
        return 1
    else:
        a, b = 0, 1
        for _ in range(2, n + 1):
            a, b = b, a + b
        return b

def is_prime(n):
    if not isinstance(n, numbers.Number) or not isinstance(n, int):
        raise TypeError("Input must be an integer.")
    if n <= 1:
        return False
    for i in range(2, int(n**0.5) + 1):
        if n % i == 0:
            return False
    return True