"""Generate the A4 marker kit using the navigation program's dictionary and IDs."""
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import cv2
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

from camera_demo import DICTIONARY_NAME, dictionary
from floor_markers import FLOOR_IDS, FLOOR_SIDE_MM, ROBOT_IDS, ROBOT_SIDE_MM

ROOT = Path(__file__).resolve().parent


def draw_page(pdf, marker_id, name, side, orientation):
    pattern = cv2.aruco.generateImageMarker(dictionary(),marker_id,6)
    pdf.setFont("Helvetica-Bold",24)
    pdf.drawCentredString(105*mm,273*mm,name)
    pdf.setLineWidth(1.5*mm)
    pdf.line(105*mm,242*mm,105*mm,260*mm)
    pdf.line(99*mm,254*mm,105*mm,260*mm)
    pdf.line(111*mm,254*mm,105*mm,260*mm)
    pdf.setFont("Helvetica",11)
    pdf.drawCentredString(105*mm,232*mm,orientation)

    cell = side*mm/6
    left = (210-side)*mm/2
    top = 219*mm
    black = pdf.beginPath()
    for row,col in zip(*((pattern==0).nonzero())):
        # A single filled vector path avoids seams at adjacent black cells.
        black.rect(left+col*cell,top-(row+1)*cell,cell,cell)
    pdf.drawPath(black,stroke=0,fill=1)

    pdf.setFont("Helvetica-Bold",11)
    pdf.drawCentredString(105*mm,40*mm,f"{DICTIONARY_NAME}   /   Black square {side} mm")
    pdf.setFont("Helvetica",10)
    pdf.drawCentredString(105*mm,32*mm,"A4   /   Actual size 100%   /   Keep the white margin")
    pdf.setLineWidth(.3*mm)
    pdf.line(55*mm,22*mm,155*mm,22*mm)
    for x in (55,155):
        pdf.line(x*mm,20*mm,x*mm,24*mm)
    pdf.drawCentredString(105*mm,15*mm,"100 mm print check")
    pdf.showPage()


def main():
    output = ROOT/"output"/"pdf"
    output.mkdir(parents=True,exist_ok=True)
    pdf_path = output/"capstone-aruco-a4.pdf"
    pdf = canvas.Canvas(str(pdf_path),pagesize=A4,pageCompression=1)
    pdf.setTitle("Capstone ArUco Marker Kit - A4")
    for marker_id in ROBOT_IDS:
        draw_page(pdf,marker_id,f"ROBOT / ID {marker_id}",ROBOT_SIDE_MM,"ROBOT FRONT")
    placements = ("TOP LEFT","TOP RIGHT","BOTTOM RIGHT","BOTTOM LEFT")
    for marker_id,placement in zip(FLOOR_IDS,placements):
        draw_page(pdf,marker_id,f"FLOOR / ID {marker_id} / {placement}",FLOOR_SIDE_MM,
                  "ARROW TOWARD THE TOP EDGE OF THE FLOOR RECTANGLE")
    pdf.save()

    zip_path = output.parent/"capstone-aruco-markers.zip"
    with ZipFile(zip_path,"w",ZIP_DEFLATED) as archive:
        archive.write(pdf_path,pdf_path.name)
        archive.write(ROOT/"FLOOR_MARKER_SETUP.md","START-HERE.md")
    print(zip_path)


if __name__ == "__main__":
    main()
