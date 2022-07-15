#!/usr/bin/python3

from rand_stu.msg import Student
from random import randint, seed, choice
from time import time


def randName():
    global names
    return choice(names)

def randLName():
    global lName
    return choice(lNames)


def randAge():
    return randint(20,70)


def randDepartement():
    global departements
    return choice(departements)


def randStudent():
    seed(time())
    student = Student()
    student.name = randName()
    student.age = randAge()

    student.last_name = randLName()
    student.departement = randDepartement()

    return student


names = [
    'Ali',
    "Mohammad",
    "Fatemeh",
    "Amir",
    "Reza",
    "Sahar",
    "Aref",
    "Aria",
    "Ahmad",
    "Akbar",
    "Mohammad Reza",
    "Amir Hosein",
    "Saman",
    "Mohsen",
    "Radin",
    "Maryam",
    "Javad",
    "Ramin",
    "Soroush",
    "Farhad",
    "Siamak",
    "Mehran",
    "Karim",
]

lNames = [
    'Akbari',
    'Hashemi',
    "Ghasemi",
    "Hoseini",
    "Eslami",
    "Kazemi",
    "Kashfi",
    "Shahi",
    "Sheikhi",
    "Kabiri",
    "Majidi",
    "Karimi",
    "Ghafori",
    "Pormokhber",
    "Ansari",
    "Modiri",
    "Fallah",
    "Ansarifard",
]


departements= [
    "Software",
    "Hardware",
]

