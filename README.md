<img width="1064" height="637" alt="image" src="https://github.com/user-attachments/assets/9c227a2b-e6fb-46ed-825e-2cbcf36f0c9b" />
Akıllı Sera Otomasyonu (Otomasyon_Sera_projesi)
Bu proje, ZEE ekibi tarafından geliştirilmiş kapsamlı bir sera otomasyon sistemidir. Projenin temel amacı, sera içerisindeki çevresel faktörleri otonom olarak izlemek ve bitki gelişimi için en ideal koşulları sağlamaktır.
Proje Özellikleri
Otomatik İklimlendirme: İçerideki sıcaklık ve nem durumuna göre havalandırma sisteminin devreye girmesi.
Akıllı Sulama: Toprak nem sensörlerinden alınan verilere göre su pompalarının otonom kontrolü.
Veri Takibi: Sensörlerden gelen veriler doğrudan kullanılmaz, kendi yazdığımız algoritmalarla hesaplanarak işlenir. Bu sayede sistem, örneğin sıcaklığın düşeceğini önceden tahmin edip tehlike anından önce alarm verir. Yani sistemimiz sadece olaylara tepki vermez, olayları önceden öngörüp koruma sağlar.
Kullanılan Teknolojiler/Donanım:

STM32F4 Discovery (veya projede kullandığın diğer mikrodenetleyiciler)
Toprak Nem Sensörü, DHT11/DHT22 Sıcaklık Sensörü vb.
Motor sürücüler ve su pompası
Yazılım:

C / C++
STM32CubeIDE / Arduino IDE
Python (Eğer veri analizi veya arayüz varsa)

