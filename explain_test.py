#code to perform addition of two vectors and normalise the result
import numpy as np

def add_vectors(v1, v2):
    return v1 + v2

def normalise_vector(v):
    return v * np.linalg.norm(v)

def main():
    v1 = np.array([1, 2, 3])
    v2 = np.array([4, 5, 6])
    v3 = add_vectors(v1, v2)
    v4 = normalise_vector(v3)
    print(v4)

if __name__ == "__main__":
    main()
    