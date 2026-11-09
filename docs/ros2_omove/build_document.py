"""Build editable Word handout from the adjacent Markdown manuscript."""
from pathlib import Path
import re
from docx import Document
from docx.shared import Cm, Pt, RGBColor
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

ROOT=Path(__file__).resolve().parent
source=next(ROOT.glob('*.md'))
doc=Document();sec=doc.sections[0]
sec.page_width=Cm(21);sec.page_height=Cm(29.7)
sec.top_margin=Cm(2.4);sec.bottom_margin=Cm(2.1);sec.left_margin=Cm(2.5);sec.right_margin=Cm(2.3)
styles=doc.styles
for name in ['Normal','Body Text','Table Grid']:
 st=styles[name];st.font.name='Noto Serif CJK SC';st.font.size=Pt(10.5)
 st.element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'),'Noto Serif CJK SC')
styles['Normal'].paragraph_format.space_after=Pt(6)
styles['Normal'].paragraph_format.line_spacing=1.35
for name,size in [('Title',24),('Heading 1',16),('Heading 2',12.5)]:
 st=styles[name];st.font.name='Noto Sans CJK SC';st.font.size=Pt(size);st.font.color.rgb=RGBColor.from_string('172B40')
 st.element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'),'Noto Sans CJK SC')
 st.paragraph_format.keep_with_next=True
code=styles.add_style('Command',1);code.font.name='DejaVu Sans Mono';code.font.size=Pt(8)
code.element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'),'Noto Sans Mono CJK SC')
code.paragraph_format.space_after=Pt(2);code.paragraph_format.line_spacing=1.1
code.paragraph_format.left_indent=Cm(.25)
header=sec.header.paragraphs[0];header.text='ROS 2 / Nav2 · OMOVE 麦克纳姆轮小车运动控制';header.style='Caption'
footer=sec.footer.paragraphs[0];footer.alignment=1
field=OxmlElement('w:fldSimple');field.set(qn('w:instr'),'PAGE');footer._p.append(field)
lines=source.read_text().splitlines();i=0;cover=True
while i<len(lines):
 line=lines[i]
 if line.startswith('# '):
  p=doc.add_paragraph(line[2:],style='Title');p.alignment=1;i+=1;continue
 if line.startswith('## '):
  if cover:
   doc.add_page_break();p=doc.add_paragraph('目　录',style='Heading 1')
   for entry in lines:
    if entry.startswith('## '):doc.add_paragraph(entry[3:],style='Normal')
   doc.add_page_break();cover=False
  elif line.startswith('## 任务') or line.startswith('## 附录'):doc.add_page_break()
  doc.add_paragraph(line[3:],style='Heading 1');i+=1;continue
 if line.startswith('### '):doc.add_paragraph(line[4:],style='Heading 2');i+=1;continue
 if line.startswith('```'):
  i+=1
  while i<len(lines) and not lines[i].startswith('```'):
   p=doc.add_paragraph(lines[i],style='Command')
   shade=OxmlElement('w:shd');shade.set(qn('w:fill'),'F2F5F7');p._p.get_or_add_pPr().append(shade)
   i+=1
  i+=1;continue
 if line.startswith('|'):
  rows=[]
  while i<len(lines) and lines[i].startswith('|'):
   cells=[x.strip() for x in lines[i].strip('|').split('|')]
   if not all(re.fullmatch(r'[:\- ]+',x) for x in cells):rows.append(cells)
   i+=1
  table=doc.add_table(rows=1,cols=len(rows[0]));table.style='Table Grid'
  for idx,row in enumerate(rows):
   cells=table.rows[0].cells if idx==0 else table.add_row().cells
   for cell,text in zip(cells,row):
    cell.text=text
    for p in cell.paragraphs:
     p.paragraph_format.line_spacing=1.15;p.paragraph_format.space_after=Pt(4)
     for r in p.runs:r.font.size=Pt(9)
    if idx==0:
     sh=OxmlElement('w:shd');sh.set(qn('w:fill'),'DCE7F0');cell._tc.get_or_add_tcPr().append(sh)
     for r in cell.paragraphs[0].runs:r.bold=True
   pr=table.rows[idx]._tr.get_or_add_trPr();pr.append(OxmlElement('w:cantSplit'))
  doc.add_paragraph();continue
 if line.startswith('!['):
  match=re.match(r'!\[(.*?)\]\((.*?)\)',line)
  doc.add_picture(str(ROOT/match.group(2)),width=Cm(16))
  p=doc.add_paragraph(match.group(1),style='Caption');p.alignment=1;i+=1;continue
 if line.strip():
  p=doc.add_paragraph(line)
  if cover:p.alignment=1;p.paragraph_format.space_after=Pt(16)
 i+=1
out=source.with_suffix('.docx');doc.save(out);print(out)
