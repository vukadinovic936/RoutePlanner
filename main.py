from flask import Flask, render_template, request, redirect, url_for
from utils import get_distance

app = Flask(__name__)
class Singleton(object):
	_instances = {}
	def __new__(class_, *args, **kwargs):
		if class_ not in class_._instances:
			class_._instances[class_] = super(Singleton, class_).__new__(class_, *args, **kwargs)
		return class_._instances[class_]

class MyClass(Singleton):
	city_names = []
	pass

@app.route('/', methods = ["GET", "POST"])
def main():
	sin = MyClass()
	if request.method == "POST":
		if request.form['submit_button'] == "add_stop":
			new_city = request.form.get("city")
			sin.city_names.append(new_city)
		elif request.form['submit_button'] == 'clear':
			print("STH ELSE")
			sin.city_names = []
		elif request.form['submit_button'] == "calculate":
			return redirect(url_for('result'))
	return render_template('index.html', city_names = sin.city_names)

@app.route('/result', methods = ["GET","POST"])
def result():

	sin = MyClass()
	dic = get_distance(sin.city_names)

	if request.method == "POST":
			return redirect(url_for('main'))

	return render_template('result.html', dic=dic)
