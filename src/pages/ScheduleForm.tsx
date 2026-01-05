/**
 * Schedule form page for planning a new mowing session.
 */

import { useState } from 'react';
import { ArrowLeft, MapPin, Calendar, Clock } from 'lucide-react';

interface ScheduleFormProps {
  onBack: () => void;
  onSubmit: (schedule: { zone: string; day: string; time: string }) => void;
}

const ZONES = ['Parcelle A1', 'Parcelle A2', 'Parcelle B1', 'Parcelle B2'];
const DAYS = ['Lundi', 'Mardi', 'Mercredi', 'Jeudi', 'Vendredi', 'Samedi', 'Dimanche'];
const TIMES = ['6h00', '7h00', '8h00', '9h00', '10h00', '11h00', '12h00', '14h00', '15h00', '16h00', '17h00', '18h00'];

export function ScheduleForm({ onBack, onSubmit }: ScheduleFormProps) {
  const [selectedZone, setSelectedZone] = useState<string | null>(null);
  const [selectedDay, setSelectedDay] = useState<string | null>(null);
  const [selectedTime, setSelectedTime] = useState<string | null>(null);

  const canSubmit = selectedZone && selectedDay && selectedTime;

  const handleSubmit = () => {
    if (canSubmit) {
      onSubmit({
        zone: selectedZone,
        day: selectedDay,
        time: selectedTime,
      });
    }
  };

  return (
    <div className="w-full max-w-md min-h-screen bg-[#d9e7f3] px-5 py-6">
      {/* Header */}
      <div className="mb-6 flex items-center gap-3">
        <button
          onClick={onBack}
          className="flex items-center justify-center w-10 h-10 rounded-xl bg-white shadow-sm"
        >
          <ArrowLeft size={20} className="text-black" />
        </button>
        <div>
          <h1 className="text-2xl font-semibold text-black">Planifier une tonte</h1>
          <p className="text-sm text-black/50">Nouvelle session</p>
        </div>
      </div>

      {/* Zone selection */}
      <div className="mb-6">
        <div className="flex items-center gap-2 mb-3">
          <MapPin size={20} className="text-black/60" />
          <span className="text-lg font-medium text-black">Parcelle</span>
        </div>
        <div className="grid grid-cols-2 gap-2">
          {ZONES.map((zone) => (
            <button
              key={zone}
              onClick={() => setSelectedZone(zone)}
              className={`p-4 rounded-xl text-left font-medium transition-all ${
                selectedZone === zone
                  ? 'bg-black text-white'
                  : 'bg-white text-black active:scale-[0.98]'
              }`}
            >
              {zone}
            </button>
          ))}
        </div>
      </div>

      {/* Day selection */}
      <div className="mb-6">
        <div className="flex items-center gap-2 mb-3">
          <Calendar size={20} className="text-black/60" />
          <span className="text-lg font-medium text-black">Jour</span>
        </div>
        <div className="flex gap-2 overflow-x-auto pb-2 -mx-5 px-5">
          {DAYS.map((day) => (
            <button
              key={day}
              onClick={() => setSelectedDay(day)}
              className={`px-4 py-3 rounded-xl font-medium whitespace-nowrap transition-all ${
                selectedDay === day
                  ? 'bg-black text-white'
                  : 'bg-white text-black active:scale-[0.98]'
              }`}
            >
              {day}
            </button>
          ))}
        </div>
      </div>

      {/* Time selection */}
      <div className="mb-8">
        <div className="flex items-center gap-2 mb-3">
          <Clock size={20} className="text-black/60" />
          <span className="text-lg font-medium text-black">Heure</span>
        </div>
        <div className="grid grid-cols-4 gap-2">
          {TIMES.map((time) => (
            <button
              key={time}
              onClick={() => setSelectedTime(time)}
              className={`p-3 rounded-xl font-medium transition-all ${
                selectedTime === time
                  ? 'bg-black text-white'
                  : 'bg-white text-black active:scale-[0.98]'
              }`}
            >
              {time}
            </button>
          ))}
        </div>
      </div>

      {/* Submit button */}
      <button
        onClick={handleSubmit}
        disabled={!canSubmit}
        className={`w-full py-4 rounded-xl font-medium text-lg transition-all ${
          canSubmit
            ? 'bg-black text-white active:scale-[0.98]'
            : 'bg-black/20 text-black/40 cursor-not-allowed'
        }`}
      >
        Planifier
      </button>

      {/* Summary */}
      {canSubmit && (
        <div className="mt-4 p-4 bg-white rounded-xl">
          <p className="text-sm text-black/50 mb-1">Résumé</p>
          <p className="text-lg font-medium text-black">
            {selectedZone} • {selectedDay} à {selectedTime}
          </p>
        </div>
      )}
    </div>
  );
}

