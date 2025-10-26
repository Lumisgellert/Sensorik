"""
3D Punktwolken Visualisierer - EINFACH
Trage einfach deinen Dateinamen unten ein und starte!
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ============================================
# HIER DEINEN DATEINAMEN EINTRAGEN:
# ============================================
DATEINAME = "scan_3d_20250126_120000.xyz"  # ← ÄNDERN!
# ============================================


def visualisiere_punktwolke(dateiname):
    """
    Lädt und zeigt 3D-Punktwolke an
    """
    print("="*60)
    print("3D Punktwolken Visualisierer")
    print("="*60)
    print()
    print(f"Lade Datei: {dateiname}")
    
    try:
        # Punktwolke laden
        points = np.loadtxt(dateiname)
        print(f"✅ {len(points)} Punkte geladen")
        print()
        
        # Statistiken anzeigen
        print("📊 Statistiken:")
        print(f"   X (vorwärts): {np.min(points[:,0]):.2f} bis {np.max(points[:,0]):.2f} m")
        print(f"   Y (seitlich): {np.min(points[:,1]):.2f} bis {np.max(points[:,1]):.2f} m")
        print(f"   Z (höhe):     {np.min(points[:,2]):.2f} bis {np.max(points[:,2]):.2f} m")
        print()
        
        # 3D Plot erstellen
        print("📈 Erstelle 3D-Visualisierung...")
        
        fig = plt.figure(figsize=(14, 10))
        
        # Plot 1: 3D-Ansicht mit Farbe nach Höhe
        ax1 = fig.add_subplot(221, projection='3d')
        scatter1 = ax1.scatter(points[:,0], points[:,1], points[:,2], 
                              c=points[:,2], cmap='viridis', s=1)
        ax1.set_xlabel('X (vorwärts) [m]')
        ax1.set_ylabel('Y (seitlich) [m]')
        ax1.set_zlabel('Z (höhe) [m]')
        ax1.set_title('3D-Ansicht (Farbe = Höhe)')
        plt.colorbar(scatter1, ax=ax1, label='Höhe [m]', shrink=0.5)
        
        # Plot 2: Draufsicht (XY)
        ax2 = fig.add_subplot(222)
        ax2.scatter(points[:,0], points[:,1], c=points[:,2], cmap='viridis', s=1)
        ax2.set_xlabel('X (vorwärts) [m]')
        ax2.set_ylabel('Y (seitlich) [m]')
        ax2.set_title('Draufsicht (von oben)')
        ax2.axis('equal')
        ax2.grid(True)
        
        # Plot 3: Seitenansicht (XZ)
        ax3 = fig.add_subplot(223)
        ax3.scatter(points[:,0], points[:,2], c=points[:,1], cmap='coolwarm', s=1)
        ax3.set_xlabel('X (vorwärts) [m]')
        ax3.set_ylabel('Z (höhe) [m]')
        ax3.set_title('Seitenansicht (von rechts)')
        ax3.axis('equal')
        ax3.grid(True)
        
        # Plot 4: Frontansicht (YZ)
        ax4 = fig.add_subplot(224)
        ax4.scatter(points[:,1], points[:,2], c=points[:,0], cmap='plasma', s=1)
        ax4.set_xlabel('Y (seitlich) [m]')
        ax4.set_ylabel('Z (höhe) [m]')
        ax4.set_title('Frontansicht (von vorne)')
        ax4.axis('equal')
        ax4.grid(True)
        
        plt.tight_layout()
        
        print("✅ Fertig! Fenster wird geöffnet...")
        print()
        print("💡 Tipps:")
        print("   - 3D-Ansicht: Drehe mit Maus")
        print("   - Zoom: Mausrad oder rechte Maustaste")
        print("   - Schließe Fenster um Programm zu beenden")
        print()
        
        plt.show()
        
        print("Fertig!")
        
    except FileNotFoundError:
        print(f"❌ Datei nicht gefunden: {dateiname}")
        print()
        print("Prüfe:")
        print(f"  1. Dateiname korrekt? (Zeile 12 in diesem Script)")
        print(f"  2. Datei im gleichen Ordner?")
        print()
        
        # Verfügbare Dateien anzeigen
        import os
        xyz_files = [f for f in os.listdir('.') if f.endswith('.xyz')]
        if xyz_files:
            print(f"📁 Gefundene .xyz Dateien in diesem Ordner:")
            for f in xyz_files:
                print(f"   - {f}")
        else:
            print("   Keine .xyz Dateien gefunden!")
        
    except Exception as e:
        print(f"❌ Fehler: {e}")


if __name__ == "__main__":
    visualisiere_punktwolke(DATEINAME)
