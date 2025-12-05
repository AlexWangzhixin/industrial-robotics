import PyPDF2

pdf_file = open('Industrial_Robotics_Assignment_2_20_11_2025.pdf', 'rb')
pdf_reader = PyPDF2.PdfReader(pdf_file)
print('Number of pages:', len(pdf_reader.pages))

for i, page in enumerate(pdf_reader.pages):
    print(f"\n=== Page {i+1} ===")
    text = page.extract_text()
    print(text)

pdf_file.close()