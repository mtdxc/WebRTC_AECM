cmake_minimum_required(VERSION 3.9)
project(aecm)

add_executable(aecm main.c aecm.c agc.c vad.c agc.c noise_suppression.c example.c)