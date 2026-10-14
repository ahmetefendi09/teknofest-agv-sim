# TEKNOFEST Sanayide Robotik Uygulamalar - AGV Simülasyonu (C++ / SFML)

TEKNOFEST AGV kategorisi için C++17 ve SFML kullanılarak sıfırdan geliştirilmiş 3 pencereli otonom fabrika taşıma, kamera vizyon ve WMS görev yönetim simülasyonu.

## Öne Çıkan Özellikler

- **3 Bağımsız Pencere Mimarisi:**
  1. **1. AGV Simülasyon Parkuru:** Serbest ve CAD/Cetvel modlu (`Shift`) çizgi çekme, dinamik istasyon, kutu ve engel yerleştirme.
  2. **2. AGV Telemetri ve Sensör Verileri:** Anlık pusula açısı, 1Hz ADC/BMS batarya voltaj ve akım modeli, kantar yükü (kg), otonom görüş güveni ve görev planı.
  3. **3. AGV WMS Terminali:** Sınırsız görev kuyruğu ayrıştırıcı (`t1k t3b / t2k t4b`) ve canlı sistem logları.
- **Aktif 3 Kameralı Navigasyon:** Ön geniş açı ($110^\circ$) + Sol ve Sağ yan kameralar ($75^\circ$) ile $90^\circ$ keskin dönüşlerde yerinde pivot dönüşü ve cep hatlarına otonom sapma.
- **Akıllı Engel Filtresi:** Siyah çizgi üzerinde olmayan engelleri yoksayar; hat üzerindeki engellerde kamera menzilinin tam yarısında (`%50`) güvenli duruş yapar.
- **Kutu Bekleme & Home Return:** İstasyonda kutu yoksa `K` ile kutu konulana kadar bekler; tüm görevler tamamlandığında otomatik olarak `HOME` park alanına döner.

## Kurulum ve Çalıştırma

### Hazır Release Paketi İle:
Release sekmesinden indirilen arşivi çıkartıp doğrudan başlatabilirsiniz:
```bash
./baslat.sh
cd ~/agv_sim

cat << 'EOF' > README.md
# TEKNOFEST Sanayide Robotik Uygulamalar - AGV Simulasyonu (C++ / SFML)

TEKNOFEST AGV kategorisi icin C++17 ve SFML kullanilarak sifirdan gelistirilmis 3 pencereli otonom fabrika tasima, kamera vizyon ve WMS gorev yonetim simulasyonu.

## One Cikan Ozellikler

- **3 Bagimsiz Pencere Mimarisi:**
  1. **1. AGV Simulasyon Parkuru:** Serbest ve CAD/Cetvel modlu (Shift) cizgi cekme, dinamik istasyon, kutu ve engel yerlestirme.
  2. **2. AGV Telemetri ve Sensor Verileri:** Anlik pusula acisi, 1Hz ADC/BMS batarya voltaj ve akim modeli, kantar yuku (kg), otonom gorus guveni ve gorev plani.
  3. **3. AGV WMS Terminali:** Sinirsiz gorev kuyrugu ayristirici (t1k t3b / t2k t4b) ve canli sistem loglari.
- **Aktif 3 Kamerali Navigasyon:** On genis aci (110 derece) + Sol ve Sag yan kameralar (75 derece) ile 90 derece keskin donuslerde yerinde pivot donusu ve cep hatlarina otonom sapma.
- **Akilli Engel Filtresi:** Siyah cizgi uzerinde olmayan engelleri yoksayar; hat uzerindeki engellerde kamera menzilinin tam yarisinda (%50) guvenli durus yapar.
- **Kutu Bekleme & Home Return:** Istasyonda kutu yoksa K ile kutu konulana kadar bekler; tum gorevler tamamlandiginda otomatik olarak HOME park alanina doner.

## Kurulum ve Calistirma

### Hazir Release Paketi Ile:
Release sekmesinden indirilen arsivi cikartip dogrudan baslatabilirsiniz:

    ./baslat.sh

### Kaynak Koddan Derleme:

    sudo dnf install SFML-devel
    make run
