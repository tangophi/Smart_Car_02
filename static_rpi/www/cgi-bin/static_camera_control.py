#!/usr/bin/python

# Import the CGI module
import cgi
import os

# Required header that tells the browser how to render the HTML.
print "Content-Type: text/html\n\n"

# Define function to generate HTML form.
def generate_form():
	print "<HTML>\n"
	print "<HEAD>\n"
	print "\t<TITLE>Info Form</TITLE>\n"
	print "</HEAD>\n"
	print "<BODY BGCOLOR = white>\n"
	print "\t\t<FORM METHOD = post ACTION =	\"static_camera_control.py\">\n"
	print "\t<INPUT TYPE = hidden NAME = \"action\" VALUE = \"display\">\n"
	print "\t<TABLE BORDER=0 align=\"center\">\n"

        print "\t<td>\n"
	print "\t<TABLE BORDER = 0>\n"
        print "\t<tr>\n"
        print "\t<td align=\"center\">\n"
	print "\t<INPUT TYPE = submit NAME = \"Button\"     VALUE = \"Camera Up\">\n"
        print "\t</td>\n"
        print "\t</tr>\n"
        print "\t<tr>\n"
        print "\t<td>\n"
	print "\t<INPUT TYPE = submit NAME = \"Button\"     VALUE = \"Camera Left\">\n"
	print "\t<INPUT TYPE = submit NAME = \"Button\"     VALUE = \"Camera Right\">\n"
        print "\t</td>\n"
        print "\t</tr>\n"
        print "\t<tr>\n"
        print "\t<td align=\"center\">\n"
	print "\t<INPUT TYPE = submit NAME = \"Button\"     VALUE = \"Camera Down\">\n"
        print "\t</td>\n"
        print "\t</tr>\n"
	print "\t</TABLE>\n"
        print "\t</td>\n"

	print "\t</TABLE>\n"
	print "\t</FORM>\n"
	print "</BODY>\n"
	print "</HTML>\n"
	
# Define main function.
def main():
	form = cgi.FieldStorage()
	if (form.has_key("action")):
		if (form["Button"].value           == "Camera Up"):
			f = os.popen('echo U > \/dev\/ttyACM0')
		if (form["Button"].value           == "Camera Down"):
			f = os.popen('echo D > \/dev\/ttyACM0')
		if (form["Button"].value           == "Camera Right"):
			f = os.popen('echo R > \/dev\/ttyACM0')
		if (form["Button"].value           == "Camera Left"):
			f = os.popen('echo L > \/dev\/ttyACM0')
	       	generate_form()
	else:
	       	generate_form()
	
# Call main function.
main()
