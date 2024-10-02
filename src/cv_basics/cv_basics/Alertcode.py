# -*- coding: utf-8 -*-
"""
Created on Mon Sep 30 14:57:02 2024

@author: cazxe

"""


import pygame


# Initialize pygame mixer
pygame.mixer.init()

# Load the MP3 file using pydub
file = "alert.wav"


    # Load the WAV file into pygame
pygame.mixer.music.load("alert.wav")

    # Play the audio
pygame.mixer.music.play()

    # Wait until the audio finishes playing
while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

print("MP3 audio playback finished.")


