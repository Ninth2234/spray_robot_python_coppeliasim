from setuptools import setup, find_packages

setup(
    name="spray_robot_python_coppeliasim",  # Package name
    version="0.1.0",  # Use semantic versioning (major.minor.patch)
    author="Kantawatchr Chaiprabha",  # Your name or organization
    author_email="kant.chai@.com",  # Contact email
    description="Spray robot simulation with Coppeliasim and remote python zmq.",  # Short description
    url="https://github.com/Ninth2234/spray_robot_python_coppeliasim.git",  # Project URL (GitHub, GitLab, etc.)
    packages=find_packages(),  # Exclude non-package folders
    python_requires=">=3.8",  # Ensure compatibility with Python 3.8+
)
