import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()


setuptools.setup(
    name='TMotorCANControl',  
    version='0.1',
    scripts=['TControl.py'] ,
    author="Mitry Anderson",
    author_email="mitryand@umich.edu",
    description="A package to manage AK series T Motors",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/mitry-anderson/TControl",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: Unix",
    ],
    python_requires=">=3.8.8"
 )