from setuptools import setup

setup(
    name="ur5_data_pipeline",
    version="0.1.0",
    packages=["ur5_data_pipeline"],
    install_requires=["SQLAlchemy", "influxdb3-python"],
)
