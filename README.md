AMARINE-ROV

This code is the intellectual property of the amarine ROBOTIIK team. All usage and distribution rights are reserved.

Project Overview

Core Functionality:
    This code facilitates autonomous drone navigation using real-time object detection for obstacle detection and for the preperation of KRBAI 2024
    
Technologies:
    pymavlink (for drone communication and control)
    YOLOv8 (for object detection)
    NVIDIA Jetson Orin Nano (for hardware acceleration)

Installation

 Prerequisites
    Python 3.x
    NVIDIA Jetpack SDK (5.1.2)
    Pytorch 
    torch
    torchvision
    
Github (sebelum mulai pastikan membuat file sendiri di laptop dan sudah install terminal bernama git bash di device sendiri):
1. Pastikan ke directory yang sudah di buat lalu buka git bash untuk melakukan git clone respository tersebut sebelum melakukan modiifkasi, untuk  commandnya yaitu : `git clone https://github.com/treeone246/rov.git`
2. checkout ke branches masing-masing yang sudah ada sensuai nama masing-masing, untuk commandnya yaitu : `git checkout <nama branch>` contoh `git checkout bhanu`
3. Modifikasi kode. Jika kode sudah benar dan akan masukkan di repository github, lakukan \`git add .\` kemudian \`git commit -m "<pesan, apa yang di modifikasi>"\` lalu \`git push\`
4. Jika ingin menggabungkan kode masing-masing pada production code (branch main) pastikan tidak ada konflik saat melakukan merge. Untuk menghindari lakukan merging dari kode main ke branch kalian `git merge main`. Jika berhasil dan ingin push ke kode main, buatlah merge request, agar dapat direview terlebih dahulu

Notes:
Sebelum melakukan modifikasi harian/daily pastikan merging terdahulu di gitbash masing-masing command: `git merge main`
