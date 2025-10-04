import unittest
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pymath.lib.math import fibonacci, is_prime

class TestFibonacci(unittest.TestCase):
    def test_fibonacci_sequence(self):
        self.assertEqual(fibonacci(0), 0)
        self.assertEqual(fibonacci(1), 1)
        self.assertEqual(fibonacci(2), 1)
        self.assertEqual(fibonacci(3), 2)
        self.assertEqual(fibonacci(4), 3)
        self.assertEqual(fibonacci(5), 5)
        self.assertEqual(fibonacci(10), 55)

    def test_negative_input(self):
        with self.assertRaises(ValueError):
            fibonacci(-1)

    def test_large_input(self):
        self.assertEqual(fibonacci(20), 6765)

    def test_type_error(self):
        with self.assertRaises(TypeError):
            fibonacci("a")
        with self.assertRaises(TypeError):
            fibonacci(1.5)

class TestIsPrime(unittest.TestCase):
    def test_prime_numbers(self):
        self.assertTrue(is_prime(2))
        self.assertTrue(is_prime(3))
        self.assertTrue(is_prime(5))
        self.assertTrue(is_prime(7))
        self.assertTrue(is_prime(11))
        self.assertTrue(is_prime(13))
        self.assertTrue(is_prime(97))

    def test_non_prime_numbers(self):
        self.assertFalse(is_prime(1))
        self.assertFalse(is_prime(4))
        self.assertFalse(is_prime(6))
        self.assertFalse(is_prime(8))
        self.assertFalse(is_prime(9))
        self.assertFalse(is_prime(10))
        self.assertFalse(is_prime(12))
        self.assertFalse(is_prime(100))

    def test_negative_numbers(self):
        self.assertFalse(is_prime(-1))
        self.assertFalse(is_prime(-2))
        self.assertFalse(is_prime(-10))

    def test_zero(self):
        self.assertFalse(is_prime(0))

    def test_type_error(self):
        with self.assertRaises(TypeError):
            is_prime("a")
        with self.assertRaises(TypeError):
            is_prime(1.5)

if __name__ == '__main__':
    unittest.main()